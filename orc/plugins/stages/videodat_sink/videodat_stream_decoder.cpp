/*
 * File:        videodat_stream_decoder.cpp
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Recovers the VideoDat byte stream from the VBI of PAL frames
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_stream_decoder.h"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <memory>
#include <set>
#include <utility>

namespace orc {

namespace {

constexpr double kFramesPerSecond = 25.0;
constexpr size_t kPalFieldLines = 313;

// Line search: blocks of a second; a line is a data line when it was framed
// in at least this fraction of the best line's fields.
constexpr uint64_t kSearchBlockFrames = 25;
constexpr uint32_t kSearchNeededHits = 100;
constexpr uint32_t kSearchStableBlocks = 5;
constexpr uint32_t kSearchMinimumHits = 10;
constexpr double kSearchSelectFraction = 0.3;

// Framing (of 20) a line needs to count as data, and the framing a line needs
// to measure the sync offset with.
constexpr uint32_t kFramedBits = 18;
constexpr uint32_t kPerfectFraming = 20;

// Sync offsets needed before the end of sync is used as timing reference.
constexpr size_t kMinimumSyncOffsets = 10;

// Equaliser training: sections of this many frames, up to this many in all,
// until this many error-free lines are in hand; below the minimum the plain
// threshold is kept.
constexpr uint64_t kTrainingChunkFrames = 1250;
constexpr uint64_t kTrainingMaximumFrames = 15000;
constexpr size_t kTrainingNeededLines = 3000;
constexpr size_t kTrainingMinimumLines = 200;

// Threshold shifts tried, in steps of the normalised swing.
constexpr int kBiasSteps = 15;
constexpr double kBiasStep = 0.05;

// Feature rows kept for training in single precision: a section of a worn tape
// can need tens of thousands of lines.
using CompactFeatures =
    std::array<float, kVideoDatBitsPerLine * kVideoDatFeatureColumns>;

CompactFeatures compact(const VideoDatLineFeatures& features) {
  CompactFeatures out{};
  for (size_t bit = 0; bit < kVideoDatBitsPerLine; ++bit) {
    for (size_t column = 0; column < kVideoDatFeatureColumns; ++column) {
      out[bit * kVideoDatFeatureColumns + column] =
          static_cast<float>(features[bit][column]);
    }
  }
  return out;
}

VideoDatLineFeatures expand(const CompactFeatures& features) {
  VideoDatLineFeatures out{};
  for (size_t bit = 0; bit < kVideoDatBitsPerLine; ++bit) {
    for (size_t column = 0; column < kVideoDatFeatureColumns; ++column) {
      out[bit][column] = features[bit * kVideoDatFeatureColumns + column];
    }
  }
  return out;
}

size_t pal_line_number(int field, size_t line) {
  return line + 1 + kPalFieldLines * static_cast<size_t>(field);
}

// Lines whose score is within reach of the best.
std::set<size_t> select_lines(const std::vector<uint32_t>& score) {
  const uint32_t best = *std::max_element(score.begin(), score.end());
  std::set<size_t> out;
  if (best < kSearchMinimumHits) return out;
  for (size_t line = 0; line < score.size(); ++line) {
    if (static_cast<double>(score[line]) >=
        static_cast<double>(best) * kSearchSelectFraction) {
      out.insert(line);
    }
  }
  return out;
}

// Lines that lie wholly inside packets whose checksum passes, and the number
// of such packets.
std::pair<std::vector<bool>, size_t> good_lines(
    const VideoDatByteStream& stream, size_t lines) {
  std::vector<bool> good(lines, false);
  size_t packets = 0;
  for (const VideoDatPacket& packet : find_videodat_packets(stream)) {
    if (!packet.ok) continue;
    const size_t first =
        (packet.position + kVideoDatBytesPerLine - 1) / kVideoDatBytesPerLine;
    const size_t end =
        (packet.position + packet.raw.size()) / kVideoDatBytesPerLine;
    for (size_t line = first; line < end && line < lines; ++line) {
      good[line] = true;
    }
    ++packets;
  }
  return {good, packets};
}

size_t passing_packets(const VideoDatByteStream& stream) {
  size_t count = 0;
  for (const VideoDatPacket& packet : find_videodat_packets(stream)) {
    if (packet.ok) ++count;
  }
  return count;
}

class Decoder {
 public:
  Decoder(const IVideoDatFrameSource& source,
          const VideoDatDecoderOptions& options,
          const VideoDatProgressFn& progress,
          const std::function<bool()>& cancelled)
      : source_(source),
        options_(options),
        progress_(progress),
        cancelled_(cancelled),
        frames_(source.frame_count()) {}

  VideoDatDecodeResult run() {
    const uint64_t scan_frames = std::min<uint64_t>(
        frames_,
        static_cast<uint64_t>(options_.scan_seconds * kFramesPerSecond));
    std::vector<double> offsets = find_lines(scan_frames);
    if (stop()) return finish_cancelled();
    if (result_.lines[0].empty() && result_.lines[1].empty()) {
      result_.log.push_back("No VideoDat lines found");
      return std::move(result_);
    }

    if (options_.use_sync && offsets.size() >= kMinimumSyncOffsets) {
      std::vector<double> sorted = offsets;
      std::sort(sorted.begin(), sorted.end());
      const size_t n = sorted.size();
      const double median = (n % 2 == 1)
                                ? sorted[n / 2]
                                : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);
      double mean = 0.0;
      for (const double o : offsets) mean += o;
      mean /= static_cast<double>(n);
      double variance = 0.0;
      for (const double o : offsets) variance += (o - mean) * (o - mean);
      result_.sync_offset = median;
      result_.log.push_back(fmt::format(
          "Start edge {:.2f} samples after the end of line sync (spread "
          "{:.2f})",
          median, std::sqrt(variance / static_cast<double>(n))));
    }

    if (options_.use_equaliser) {
      train(std::min<uint64_t>(scan_frames, frames_ / 2));
      if (stop()) return finish_cancelled();
    }

    decode();
    if (stop()) return finish_cancelled();
    return std::move(result_);
  }

 private:
  bool stop() const { return cancelled_ && cancelled_(); }

  VideoDatDecodeResult finish_cancelled() {
    result_.cancelled = true;
    return std::move(result_);
  }

  void report(uint64_t done, uint64_t total, const std::string& message) {
    if (progress_) progress_(done, total, message);
  }

  bool read(uint64_t frame, int field, size_t line) {
    return source_.read_line(frame, field, line, samples_) && !samples_.empty();
  }

  std::optional<double> prior() const {
    if (!result_.sync_offset.has_value()) return std::nullopt;
    const auto end =
        videodat_sync_end(samples_.data(), samples_.size(), options_.slicer);
    if (!end.has_value()) return std::nullopt;
    return *end + *result_.sync_offset;
  }

  // Count framed lines per field parity; returns the sync offsets of the
  // perfectly framed lines among the chosen ones.
  std::vector<double> find_lines(uint64_t scan_frames) {
    std::array<std::vector<uint32_t>, 2> score{
        std::vector<uint32_t>(kVideoDatSearchLines, 0),
        std::vector<uint32_t>(kVideoDatSearchLines, 0)};
    struct Offset {
      int field;
      size_t line;
      double offset;
    };
    std::vector<Offset> offsets;
    std::array<std::set<size_t>, 2> last;
    bool have_last = false;
    uint32_t stable = 0;
    uint64_t scanned = 0;

    for (uint64_t block = 0; block < scan_frames; block += kSearchBlockFrames) {
      const uint64_t block_end =
          std::min(scan_frames, block + kSearchBlockFrames);
      for (uint64_t frame = block; frame < block_end; ++frame) {
        for (int field = 0; field < 2; ++field) {
          for (size_t line = 0; line < kVideoDatSearchLines; ++line) {
            if (!read(frame, field, line)) continue;
            const auto phase = videodat_line_phase(
                samples_.data(), samples_.size(), options_.slicer);
            if (!phase.has_value()) continue;
            const uint32_t framing = videodat_line_framing(
                samples_.data(), samples_.size(), *phase, options_.slicer);
            if (framing >= kFramedBits) ++score[field][line];
            if (framing == kPerfectFraming) {
              const auto end = videodat_sync_end(
                  samples_.data(), samples_.size(), options_.slicer);
              if (end.has_value()) {
                offsets.push_back({field, line, phase->start_edge - *end});
              }
            }
          }
        }
      }
      scanned = block_end;
      report(scanned, scan_frames, "Searching VideoDat lines");
      if (stop()) return {};

      const std::array<std::set<size_t>, 2> sets{select_lines(score[0]),
                                                 select_lines(score[1])};
      const uint32_t weakest =
          std::min(*std::max_element(score[0].begin(), score[0].end()),
                   *std::max_element(score[1].begin(), score[1].end()));
      stable = (weakest >= kSearchNeededHits && have_last && sets == last)
                   ? stable + 1
                   : 0;
      last = sets;
      have_last = true;
      if (stable >= kSearchStableBlocks) break;
    }

    std::string hits;
    for (int field = 0; field < 2; ++field) {
      for (const size_t line : last[field]) {
        if (!hits.empty()) hits += ", ";
        hits += fmt::format("{}:{}", pal_line_number(field, line),
                            score[field][line]);
      }
    }
    result_.log.push_back(
        fmt::format("Line search over {} frames ({:.1f} s), hits per line: {}",
                    scanned, static_cast<double>(scanned) / kFramesPerSecond,
                    hits.empty() ? "-" : hits));
    std::string chosen;
    for (int field = 0; field < 2; ++field) {
      result_.lines[field].assign(last[field].begin(), last[field].end());
      for (const size_t line : last[field]) {
        if (!chosen.empty()) chosen += ", ";
        chosen += fmt::format("{}", pal_line_number(field, line));
      }
      if (last[field].empty()) {
        result_.log.push_back(fmt::format(
            "WARNING: no VideoDat line found in field {}", field + 1));
      }
    }
    if (!chosen.empty()) {
      result_.log.push_back(fmt::format("VideoDat lines (PAL): {}", chosen));
    }

    std::vector<double> out;
    for (const Offset& o : offsets) {
      if (last[o.field].count(o.line) != 0) out.push_back(o.offset);
    }
    return out;
  }

  void train(uint64_t start) {
    std::vector<CompactFeatures> features;
    std::vector<bool> good;
    size_t simple_packets = 0;
    const uint64_t end = std::min(frames_, start + kTrainingMaximumFrames);
    uint64_t frame = start;
    while (frame < end) {
      const uint64_t chunk_end = std::min(frame + kTrainingChunkFrames, end);
      for (; frame < chunk_end; ++frame) {
        for (int field = 0; field < 2; ++field) {
          for (const size_t line : result_.lines[field]) {
            if (!read(frame, field, line)) continue;
            const auto phase = videodat_line_phase(
                samples_.data(), samples_.size(), options_.slicer, prior());
            if (!phase.has_value()) continue;
            features.push_back(compact(videodat_line_features(
                samples_.data(), samples_.size(), *phase, options_.slicer)));
          }
        }
        if (stop()) return;
      }
      report(frame - start, end - start, "Training the equaliser");
      VideoDatByteStream simple;
      for (const CompactFeatures& f : features) {
        simple.append(videodat_slice_features(expand(f)));
      }
      std::tie(good, simple_packets) = good_lines(simple, features.size());
      if (static_cast<size_t>(std::count(good.begin(), good.end(), true)) >=
          kTrainingNeededLines) {
        break;
      }
    }

    const size_t good_count =
        static_cast<size_t>(std::count(good.begin(), good.end(), true));
    if (good_count < kTrainingMinimumLines) {
      result_.log.push_back(fmt::format(
          "Equaliser: too few error-free lines ({}), using the plain "
          "threshold",
          good_count));
      return;
    }

    std::vector<std::unique_ptr<VideoDatLineFeatures>> expanded;
    std::vector<const VideoDatLineFeatures*> rows;
    for (size_t i = 0; i < features.size(); ++i) {
      if (!good[i]) continue;
      expanded.push_back(
          std::make_unique<VideoDatLineFeatures>(expand(features[i])));
      rows.push_back(expanded.back().get());
    }
    std::optional<VideoDatEqualiser> equaliser = videodat_fit_equaliser(rows);
    expanded.clear();
    if (!equaliser.has_value()) {
      result_.log.push_back("Equaliser: fit failed, using the plain threshold");
      return;
    }

    // Threshold shift: the most passing checksums in the training section.
    std::vector<double> hits(kBiasSteps, 0.0);
    for (int step = 0; step < kBiasSteps; ++step) {
      VideoDatByteStream trial;
      for (const CompactFeatures& f : features) {
        trial.append(
            videodat_slice_features(expand(f), &*equaliser, kBiasStep * step));
      }
      hits[static_cast<size_t>(step)] =
          static_cast<double>(passing_packets(trial));
      if (stop()) return;
    }
    // Lightly smoothed: the optimum falls off steeply towards large shifts.
    size_t best = 0;
    double best_value = -1.0;
    for (size_t k = 0; k < hits.size(); ++k) {
      const double before = hits[k == 0 ? 0 : k - 1];
      const double after = hits[std::min(k + 1, hits.size() - 1)];
      const double value = (before + hits[k] + after) / 3.0;
      if (value > best_value) {
        best_value = value;
        best = k;
      }
    }
    equaliser->bias = kBiasStep * static_cast<double>(best);
    result_.equaliser = equaliser;
    result_.log.push_back(fmt::format(
        "Equaliser trained on {} error-free lines (frames {}-{}), threshold "
        "shift {:.2f}: {} instead of {} error-free packets in the training "
        "section",
        good_count, start, frame, equaliser->bias,
        static_cast<size_t>(hits[best]), simple_packets));
  }

  struct LineState {
    std::optional<VideoDatLinePhase> phase;
    uint32_t framing = 0;
    bool read = false;
  };

  // Lines of one field to decode, for input without sync.  When every
  // expected line is framed they are kept; otherwise the group is moved
  // within the field to where most of its lines are framed (the smallest
  // move among equals), and after a move only framed lines are used, so no
  // picture content reaches the stream.
  std::vector<std::pair<size_t, std::optional<VideoDatLinePhase>>> track(
      uint64_t frame, int field, const std::vector<size_t>& nominal) {
    std::map<size_t, LineState> cache;
    const auto get = [&](size_t line) -> const LineState& {
      auto [it, inserted] = cache.try_emplace(line);
      if (inserted) {
        LineState& state = it->second;
        if (read(frame, field, line)) {
          state.read = true;
          state.phase = videodat_line_phase(samples_.data(), samples_.size(),
                                            options_.slicer);
          if (state.phase.has_value()) {
            state.framing =
                videodat_line_framing(samples_.data(), samples_.size(),
                                      *state.phase, options_.slicer);
          }
        }
      }
      return it->second;
    };
    const auto framed = [&](size_t line) {
      const LineState& state = get(line);
      return state.phase.has_value() && state.framing >= kFramedBits;
    };

    std::vector<std::pair<size_t, std::optional<VideoDatLinePhase>>> out;
    if (std::all_of(nominal.begin(), nominal.end(), framed)) {
      for (const size_t line : nominal) out.emplace_back(line, get(line).phase);
      shifted_ = false;
      return out;
    }

    const int64_t lo = 0;
    const int64_t hi = static_cast<int64_t>(kVideoDatSearchLines);
    const int64_t first = static_cast<int64_t>(nominal.front());
    const int64_t last = static_cast<int64_t>(nominal.back());
    std::vector<int64_t> shifts;
    for (int64_t d = lo - last; d < hi - first; ++d) shifts.push_back(d);
    std::stable_sort(shifts.begin(), shifts.end(), [](int64_t a, int64_t b) {
      return std::llabs(a) < std::llabs(b);
    });
    const auto framed_at = [&](int64_t d) {
      std::vector<size_t> lines;
      for (const size_t line : nominal) {
        const int64_t moved = static_cast<int64_t>(line) + d;
        if (moved >= lo && moved < hi && framed(static_cast<size_t>(moved))) {
          lines.push_back(static_cast<size_t>(moved));
        }
      }
      return lines;
    };
    size_t best_count = 0;
    int64_t best_shift = 0;
    for (const int64_t d : shifts) {
      const size_t count = framed_at(d).size();
      if (count > best_count) {
        best_count = count;
        best_shift = d;
      }
    }
    shifted_ = best_shift != 0;
    if (best_shift == 0) {
      for (const size_t line : nominal) out.emplace_back(line, get(line).phase);
      return out;
    }
    for (const size_t line : framed_at(best_shift)) {
      out.emplace_back(line, get(line).phase);
    }
    return out;
  }

  void decode() {
    const bool tracking = options_.track_lines && !result_.sync_offset;
    const VideoDatEqualiser* equaliser =
        result_.equaliser.has_value() ? &*result_.equaliser : nullptr;
    size_t data_lines = 0;
    size_t synced_lines = 0;
    size_t shifted_fields = 0;
    const uint64_t report_every = 250;

    for (uint64_t frame = 0; frame < frames_; ++frame) {
      for (int field = 0; field < 2; ++field) {
        const std::vector<size_t>& nominal = result_.lines[field];
        if (nominal.empty()) continue;
        std::vector<std::pair<size_t, std::optional<VideoDatLinePhase>>> lines;
        if (tracking) {
          lines = track(frame, field, nominal);
          if (shifted_) ++shifted_fields;
        } else {
          for (const size_t line : nominal) {
            lines.emplace_back(line, std::nullopt);
          }
        }
        for (const auto& [line, known_phase] : lines) {
          if (!read(frame, field, line)) continue;
          std::optional<VideoDatLinePhase> phase = known_phase;
          if (!phase.has_value()) {
            const std::optional<double> reference = prior();
            if (reference.has_value()) ++synced_lines;
            phase = videodat_line_phase(samples_.data(), samples_.size(),
                                        options_.slicer, reference);
          }
          if (!phase.has_value()) continue;  // nothing transmitted
          result_.stream.append(videodat_slice_features(
              videodat_line_features(samples_.data(), samples_.size(), *phase,
                                     options_.slicer),
              equaliser));
          ++data_lines;
        }
      }
      if ((frame + 1) % report_every == 0 || frame + 1 == frames_) {
        report(frame + 1, frames_, "Decoding VideoDat lines");
        if (stop()) return;
      }
    }

    result_.log.push_back(fmt::format(
        "{} frames, {} data lines{} -> {} bytes", frames_, data_lines,
        result_.sync_offset.has_value()
            ? fmt::format(" ({} with sync reference)", synced_lines)
            : std::string(),
        result_.stream.data.size()));
    if (shifted_fields > 0) {
      result_.log.push_back(fmt::format(
          "  the VideoDat lines were moved in {} fields", shifted_fields));
    }
  }

  const IVideoDatFrameSource& source_;
  const VideoDatDecoderOptions& options_;
  const VideoDatProgressFn& progress_;
  const std::function<bool()>& cancelled_;
  const uint64_t frames_;
  std::vector<int16_t> samples_;
  bool shifted_ = false;
  VideoDatDecodeResult result_;
};

}  // namespace

VideoDatDecodeResult decode_videodat_stream(
    const IVideoDatFrameSource& source, const VideoDatDecoderOptions& options,
    const VideoDatProgressFn& progress,
    const std::function<bool()>& cancelled) {
  Decoder decoder(source, options, progress, cancelled);
  return decoder.run();
}

}  // namespace orc
