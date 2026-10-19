/*
 * File:        vbi_start_edge_locator.cpp
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Locates the first start bit of an asynchronously framed line
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "vbi_start_edge_locator.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace orc {

namespace {

// An asynchronously framed byte: one start bit, eight data bits, one stop bit.
constexpr uint32_t kFramedBitsPerByte = 10;

// Swing below which a line is not a data line.  The 8-bit figure is the
// reference decoder's for bt8x8 captures; the 16-bit one is its figure for a
// decoder's .tbc, whose data swing on the reference crop is about 27 000.
constexpr double kMinimumSwingU8 = 30.0;
constexpr double kMinimumSwing16Bit = 3000.0;

// Bit periods ahead of the search window read as the line's logic 0.
constexpr double kQuietBits = 4.0;

// Bit periods from the window's start over which logic 1 is read.  A little
// longer than a line, so a line that starts late in the window is covered.
constexpr double kPeakBits = 105.0;

// Upper decile of the data region: inside the one bits, clear of single
// overshoots.
constexpr double kPeakQuantile = 0.9;

// Samples behind the coarse edge from which crossings are folded into the
// phase estimate, so the coarse edge's own crossing is among them.
constexpr double kPhaseLeadSamples = 3.0;

// Crossings below which a phase estimate is not taken: too few transitions to
// average, and the coarse edge stands.
constexpr size_t kMinimumCrossings = 5;

constexpr double kTwoPi = 6.283185307179586;

double quantile_of(std::vector<double> values, double quantile) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const double position = quantile * static_cast<double>(values.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(position));
  const size_t upper = std::min(lower + 1, values.size() - 1);
  const double fraction = position - std::floor(position);
  return values[lower] + (values[upper] - values[lower]) * fraction;
}

std::vector<double> span(const std::vector<double>& samples, double begin,
                         double end) {
  const double size = static_cast<double>(samples.size());
  const size_t first =
      static_cast<size_t>(std::clamp(std::floor(begin), 0.0, size));
  const size_t last =
      static_cast<size_t>(std::clamp(std::ceil(end), 0.0, size));
  if (last <= first) {
    return {};
  }
  return std::vector<double>(samples.begin() + static_cast<ptrdiff_t>(first),
                             samples.begin() + static_cast<ptrdiff_t>(last));
}

}  // namespace

double vbi_start_edge_minimum_swing(VBISampleFormat sample_format) {
  return (sample_format == VBISampleFormat::kU8) ? kMinimumSwingU8
                                                 : kMinimumSwing16Bit;
}

uint32_t vbi_line_framing_bits(const std::vector<double>& record_samples,
                               double start_edge_samples, double threshold,
                               double samples_per_bit, uint32_t bytes,
                               uint32_t bits_per_byte) {
  if (bytes == 0 || bits_per_byte < 2 || !(samples_per_bit > 0.0)) {
    return 0;
  }
  const auto level_at = [&](double position, bool& high) {
    if (!(position >= 0.0)) {
      return false;
    }
    const size_t index = static_cast<size_t>(position);
    if (index >= record_samples.size()) {
      return false;
    }
    high = record_samples[index] >= threshold;
    return true;
  };

  uint32_t agreeing = 0;
  for (uint32_t byte = 0; byte < bytes; ++byte) {
    const double first_bit = static_cast<double>(byte * bits_per_byte);
    bool start_high = false;
    bool stop_high = false;
    if (level_at(start_edge_samples + (first_bit + 0.5) * samples_per_bit,
                 start_high) &&
        start_high) {
      ++agreeing;
    }
    if (level_at(start_edge_samples +
                     (first_bit + static_cast<double>(bits_per_byte) - 0.5) *
                         samples_per_bit,
                 stop_high) &&
        !stop_high) {
      ++agreeing;
    }
  }
  const uint32_t total = bytes * 2u;
  return std::max(agreeing, total - agreeing);
}

VBICRIDetection detect_vbi_start_edge_position(
    const std::vector<double>& record_samples,
    const VBITeletextService& service, double sample_rate_hz,
    const VBICRISearchWindow& window, const VBIStartEdgeConfig& config) {
  VBICRIDetection detection;

  const double samples_per_bit = service.samples_per_bit(sample_rate_hz);
  const uint32_t line_bits = service.transmitted_bits();
  const uint32_t bytes = line_bits / kFramedBitsPerByte;
  const size_t count = record_samples.size();
  if (!(samples_per_bit > 0.0) || bytes == 0 || count < 2 || window.empty()) {
    return detection;
  }

  // Levels: logic 0 from just ahead of the window, logic 1 from the data.
  const std::vector<double> quiet =
      span(record_samples, window.begin_samples - kQuietBits * samples_per_bit,
           window.begin_samples);
  const std::vector<double> data =
      span(record_samples, window.begin_samples,
           window.begin_samples + kPeakBits * samples_per_bit);
  if (quiet.empty() || data.empty()) {
    return detection;
  }
  const double low = quantile_of(quiet, 0.5);
  const double high = quantile_of(data, kPeakQuantile);
  if (high - low < config.minimum_swing_counts) {
    return detection;
  }
  const double threshold = 0.5 * (low + high);

  // Coarse edge: the first upward crossing of the threshold inside the window.
  const size_t search_begin = static_cast<size_t>(
      std::max(1.0, std::floor(window.begin_samples) + 1.0));
  const size_t search_end = static_cast<size_t>(
      std::min(static_cast<double>(count), std::ceil(window.end_samples)));
  double edge = -1.0;
  for (size_t index = search_begin; index < search_end; ++index) {
    const double before = record_samples[index - 1];
    const double after = record_samples[index];
    if (before < threshold && threshold <= after) {
      edge = static_cast<double>(index - 1) +
             (threshold - before) / (after - before);
      break;
    }
  }
  if (edge < 0.0) {
    return detection;
  }

  // Fine phase: every transition of the line, folded onto the bit period.
  const size_t fold_begin =
      static_cast<size_t>(std::max(0.0, std::floor(edge - kPhaseLeadSamples)));
  const size_t fold_end = static_cast<size_t>(
      std::min(static_cast<double>(count - 1),
               std::floor(edge + (static_cast<double>(line_bits) + 1.0) *
                                     samples_per_bit)));
  std::complex<double> phasor(0.0, 0.0);
  size_t crossings = 0;
  for (size_t index = fold_begin; index < fold_end; ++index) {
    const double here = record_samples[index];
    const double next = record_samples[index + 1];
    if ((here >= threshold) == (next >= threshold)) {
      continue;
    }
    const double position =
        static_cast<double>(index) + (threshold - here) / (next - here);
    phasor += std::polar(1.0, kTwoPi * (position - edge) / samples_per_bit);
    ++crossings;
  }
  if (crossings >= kMinimumCrossings) {
    edge += std::arg(phasor) * samples_per_bit / kTwoPi;
  }
  if (!std::isfinite(edge) || edge < 0.0 ||
      edge + static_cast<double>(line_bits) * samples_per_bit >=
          static_cast<double>(count - 1)) {
    return detection;
  }

  const uint32_t framing =
      vbi_line_framing_bits(record_samples, edge, threshold, samples_per_bit,
                            bytes, kFramedBitsPerByte);

  detection.anchor_position_samples = edge;
  detection.peak_lag = static_cast<int64_t>(edge);
  detection.peak_correlation =
      static_cast<double>(framing) / static_cast<double>(bytes * 2u);
  detection.refined = crossings >= kMinimumCrossings;
  detection.refinement_samples = edge - std::floor(edge);
  detection.accepted = framing >= config.minimum_framing_bits;
  return detection;
}

}  // namespace orc
