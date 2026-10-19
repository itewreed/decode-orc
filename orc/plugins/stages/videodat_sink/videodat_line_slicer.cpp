/*
 * File:        videodat_line_slicer.cpp
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Bit slicing of Pro7 Channel VideoDat lines on 4fsc PAL frames
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_line_slicer.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace orc {

namespace {

constexpr double kTwoPi = 6.283185307179586;

// Bit periods after the start of the search window over which logic 1 is read:
// a little more than a line, so a line starting late in the window is covered.
constexpr double kPeakBits = 105.0;

// Upper decile of the data region: inside the one bits, clear of overshoot.
constexpr double kPeakQuantile = 0.9;

// Samples behind the start edge from which transitions are folded into the
// phase estimate, so the edge's own transition is among them.
constexpr size_t kPhaseLeadSamples = 3;

// Transitions a phase estimate needs before it replaces the coarse edge.
constexpr size_t kMinimumTransitions = 5;

// Iterations of the two-means threshold; it settles in two or three.
constexpr int kTwoMeansIterations = 5;

// Equaliser taps are spaced half a bit apart around each bit centre.
constexpr double kTapSpacingBits = 0.5;

double quantile_of(std::vector<double> values, double quantile) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const double position = quantile * static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(position));
  const size_t upper = std::min(lower + 1, values.size() - 1);
  const double fraction = position - std::floor(position);
  return values[lower] + (values[upper] - values[lower]) * fraction;
}

std::vector<double> window_of(const int16_t* samples, size_t count,
                              size_t begin, size_t end) {
  end = std::min(end, count);
  if (begin >= end) return {};
  return std::vector<double>(samples + begin, samples + end);
}

// Linear interpolation at a fractional position, clamped to the line.
double sample_at(const int16_t* samples, size_t count, double position) {
  const double last = static_cast<double>(count) - 2.0;
  const double clamped = std::clamp(position, 0.0, std::max(0.0, last));
  const size_t index = static_cast<size_t>(clamped);
  const double fraction = clamped - static_cast<double>(index);
  const double here = samples[index];
  const double next = (index + 1 < count) ? samples[index + 1] : here;
  return here * (1.0 - fraction) + next * fraction;
}

}  // namespace

std::optional<VideoDatLinePhase> videodat_line_phase(
    const int16_t* samples, size_t count, const VideoDatSlicerConfig& config,
    std::optional<double> prior) {
  if (samples == nullptr || count < config.search_begin + 2) {
    return std::nullopt;
  }
  const double spb = config.samples_per_bit();

  VideoDatLinePhase phase;
  phase.low = quantile_of(
      window_of(samples, count, config.quiet_begin, config.quiet_end), 0.5);
  phase.high = quantile_of(
      window_of(samples, count, config.search_begin,
                config.search_begin + static_cast<size_t>(kPeakBits * spb)),
      kPeakQuantile);
  if (phase.high - phase.low < config.minimum_swing) {
    return std::nullopt;
  }
  phase.threshold = 0.5 * (phase.low + phase.high);
  const double threshold = phase.threshold;

  double edge = 0.0;
  if (prior.has_value()) {
    edge = *prior;
  } else {
    // The first genuine rising edge: the sample before is below the threshold
    // and the sample after at or above it.
    const size_t end = std::min(config.search_end, count);
    size_t index = config.search_begin + 1;
    while (index < end &&
           !(samples[index - 1] < threshold && threshold <= samples[index])) {
      ++index;
    }
    if (index >= end) {
      return std::nullopt;
    }
    const double before = samples[index - 1];
    const double after = samples[index];
    edge = static_cast<double>(index - 1) +
           (threshold - before) / (after - before);
  }

  // Refine the phase over every transition of the line: each crossing is
  // folded onto the bit period, and their mean direction is where all of them
  // agree the bit boundaries are.
  if (std::isfinite(edge)) {
    const size_t begin = static_cast<size_t>(std::max(
        0.0, std::floor(edge) - static_cast<double>(kPhaseLeadSamples)));
    const size_t end = std::min(
        begin + static_cast<size_t>(
                    static_cast<double>(kVideoDatBitsPerLine + 1) * spb),
        count - 2);
    std::complex<double> phasor(0.0, 0.0);
    size_t transitions = 0;
    for (size_t index = begin; index < end; ++index) {
      const bool here = samples[index] >= threshold;
      const bool next = samples[index + 1] >= threshold;
      if (here == next) continue;
      const double position =
          static_cast<double>(index) +
          (threshold - samples[index]) /
              static_cast<double>(samples[index + 1] - samples[index]);
      phasor += std::polar(1.0, kTwoPi * (position - edge) / spb);
      ++transitions;
    }
    if (transitions >= kMinimumTransitions) {
      edge += std::arg(phasor) * spb / kTwoPi;
    }
  }

  if (!std::isfinite(edge) || edge < 0.0 ||
      edge + static_cast<double>(kVideoDatBitsPerLine) * spb >=
          static_cast<double>(count) - 1.0) {
    return std::nullopt;
  }
  phase.start_edge = edge;
  return phase;
}

uint32_t videodat_line_framing(const int16_t* samples, size_t count,
                               const VideoDatLinePhase& phase,
                               const VideoDatSlicerConfig& config) {
  const double spb = config.samples_per_bit();
  uint32_t agreeing = 0;
  for (size_t byte = 0; byte < kVideoDatBytesPerLine; ++byte) {
    const double first = static_cast<double>(byte * kVideoDatBitsPerByte);
    const size_t start =
        static_cast<size_t>(phase.start_edge + (first + 0.5) * spb);
    const size_t stop = static_cast<size_t>(
        phase.start_edge +
        (first + static_cast<double>(kVideoDatBitsPerByte) - 0.5) * spb);
    if (start < count && samples[start] >= phase.threshold) ++agreeing;
    if (stop < count && samples[stop] < phase.threshold) ++agreeing;
  }
  const uint32_t total = static_cast<uint32_t>(2 * kVideoDatBytesPerLine);
  return std::max(agreeing, total - agreeing);
}

std::optional<double> videodat_sync_end(const int16_t* samples, size_t count,
                                        const VideoDatSlicerConfig& config) {
  if (samples == nullptr || count < config.sync_blank_end ||
      count < config.sync_edge_end + 1) {
    return std::nullopt;
  }
  const double tip = quantile_of(
      window_of(samples, count, config.sync_tip_begin, config.sync_tip_end),
      0.5);
  const double blank = quantile_of(
      window_of(samples, count, config.sync_blank_begin, config.sync_blank_end),
      0.5);
  if (blank - tip < config.minimum_sync_amplitude) {
    return std::nullopt;
  }
  const double middle = 0.5 * (tip + blank);
  for (size_t index = config.sync_edge_begin; index + 1 < config.sync_edge_end;
       ++index) {
    if (samples[index] < middle && samples[index + 1] >= middle) {
      return static_cast<double>(index) +
             (middle - samples[index]) /
                 static_cast<double>(samples[index + 1] - samples[index]);
    }
  }
  return std::nullopt;
}

VideoDatLineFeatures videodat_line_features(
    const int16_t* samples, size_t count, const VideoDatLinePhase& phase,
    const VideoDatSlicerConfig& config) {
  const double spb = config.samples_per_bit();
  VideoDatLineFeatures features{};

  std::array<double, kVideoDatBitsPerLine> centres{};
  for (size_t bit = 0; bit < kVideoDatBitsPerLine; ++bit) {
    for (size_t tap = 0; tap < kVideoDatEqualiserTaps; ++tap) {
      const double offset =
          (static_cast<double>(tap) - static_cast<double>(kVideoDatCentreTap)) *
          kTapSpacingBits;
      features[bit][tap] = sample_at(
          samples, count,
          phase.start_edge + (static_cast<double>(bit) + 0.5 + offset) * spb);
    }
    centres[bit] = features[bit][kVideoDatCentreTap];
  }

  // Two-means threshold over the bit centres.
  double threshold = phase.threshold;
  for (int iteration = 0; iteration < kTwoMeansIterations; ++iteration) {
    double low_sum = 0.0;
    double high_sum = 0.0;
    size_t low_count = 0;
    size_t high_count = 0;
    for (const double centre : centres) {
      if (centre < threshold) {
        low_sum += centre;
        ++low_count;
      } else {
        high_sum += centre;
        ++high_count;
      }
    }
    if (low_count == 0 || high_count == 0) break;
    threshold = 0.5 * (low_sum / static_cast<double>(low_count) +
                       high_sum / static_cast<double>(high_count));
  }

  const double swing = phase.high - phase.low;
  for (auto& row : features) {
    for (size_t tap = 0; tap < kVideoDatEqualiserTaps; ++tap) {
      row[tap] = (row[tap] - threshold) / swing;
    }
    row[kVideoDatEqualiserTaps] = 1.0;
  }
  return features;
}

VideoDatSlicedLine videodat_line_from_decisions(
    const std::array<double, kVideoDatBitsPerLine>& decisions) {
  VideoDatSlicedLine line;
  for (size_t byte = 0; byte < kVideoDatBytesPerLine; ++byte) {
    uint8_t value = 0;
    for (size_t bit = 0; bit < kVideoDatDataBitsPerByte; ++bit) {
      // Data bits follow the start bit, least significant first.
      const double decision = decisions[byte * kVideoDatBitsPerByte + 1 + bit];
      line.soft[byte * kVideoDatDataBitsPerByte + bit] =
          static_cast<float>(decision);
      if (decision >= 0.0) {
        value = static_cast<uint8_t>(value | (1u << bit));
      }
    }
    // Every byte is transmitted inverted.
    line.bytes[byte] = static_cast<uint8_t>(value ^ 0xFFu);
  }
  return line;
}

VideoDatSlicedLine videodat_slice_features(const VideoDatLineFeatures& features,
                                           const VideoDatEqualiser* equaliser,
                                           double extra_bias) {
  std::array<double, kVideoDatBitsPerLine> decisions{};
  for (size_t bit = 0; bit < kVideoDatBitsPerLine; ++bit) {
    if (equaliser == nullptr) {
      decisions[bit] = features[bit][kVideoDatCentreTap] + extra_bias;
      continue;
    }
    double value = equaliser->bias + extra_bias;
    for (size_t column = 0; column < kVideoDatFeatureColumns; ++column) {
      value += features[bit][column] * equaliser->weights[column];
    }
    decisions[bit] = value;
  }
  return videodat_line_from_decisions(decisions);
}

std::optional<VideoDatEqualiser> videodat_fit_equaliser(
    const std::vector<const VideoDatLineFeatures*>& good_lines) {
  // Normal equations of the least-squares fit of every bit's feature row to
  // the +1 / -1 its centre tap decided.  Ten unknowns, so they are solved
  // directly.
  constexpr size_t kN = kVideoDatFeatureColumns;
  std::array<std::array<double, kN + 1>, kN> system{};
  size_t rows = 0;
  for (const VideoDatLineFeatures* line : good_lines) {
    if (line == nullptr) continue;
    for (const auto& row : *line) {
      const double target = (row[kVideoDatCentreTap] >= 0.0) ? 1.0 : -1.0;
      for (size_t i = 0; i < kN; ++i) {
        for (size_t j = 0; j < kN; ++j) {
          system[i][j] += row[i] * row[j];
        }
        system[i][kN] += row[i] * target;
      }
      ++rows;
    }
  }
  if (rows < kN) return std::nullopt;

  // Gaussian elimination with partial pivoting.
  for (size_t column = 0; column < kN; ++column) {
    size_t pivot = column;
    for (size_t row = column + 1; row < kN; ++row) {
      if (std::fabs(system[row][column]) > std::fabs(system[pivot][column])) {
        pivot = row;
      }
    }
    if (std::fabs(system[pivot][column]) < 1e-12) return std::nullopt;
    std::swap(system[column], system[pivot]);
    for (size_t row = 0; row < kN; ++row) {
      if (row == column) continue;
      const double factor = system[row][column] / system[column][column];
      for (size_t k = column; k <= kN; ++k) {
        system[row][k] -= factor * system[column][k];
      }
    }
  }

  VideoDatEqualiser equaliser;
  for (size_t i = 0; i < kN; ++i) {
    equaliser.weights[i] = system[i][kN] / system[i][i];
  }
  return equaliser;
}

}  // namespace orc
