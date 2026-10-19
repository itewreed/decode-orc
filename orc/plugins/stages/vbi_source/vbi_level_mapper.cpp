/*
 * File:        vbi_level_mapper.cpp
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Maps card-capture sample levels into the CVBS amplitude domain
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 Simon Inns
 */

#include "vbi_level_mapper.h"

#include <algorithm>
#include <cmath>

namespace orc {

namespace {

// Quantile of the clock run-in window taken as its peak level.  The run-in is
// an even alternation of ones and zeros, so its upper decile sits inside the
// one bits while staying clear of any single noisy peak.
constexpr double kCRIPeakQuantile = 0.9;

// Linear-interpolated quantile of an already sorted, non-empty sample set.
double sorted_quantile(const std::vector<double>& sorted, double quantile) {
  if (sorted.empty()) {
    return 0.0;
  }
  if (sorted.size() == 1) {
    return sorted.front();
  }

  const double position = quantile * static_cast<double>(sorted.size() - 1);
  const double floor_position = std::floor(position);
  const size_t lower = static_cast<size_t>(floor_position);
  const size_t upper = std::min(lower + 1, sorted.size() - 1);
  const double fraction = position - floor_position;
  return sorted[lower] + (sorted[upper] - sorted[lower]) * fraction;
}

double median_of(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  return sorted_quantile(values, 0.5);
}

// Copy a half-open window of a record, clamped to the samples present.
std::vector<double> window_samples(const std::vector<double>& samples,
                                   uint32_t begin, uint32_t end) {
  const size_t count = samples.size();
  const size_t first = std::min(static_cast<size_t>(begin), count);
  const size_t last = std::min(static_cast<size_t>(end), count);
  if (last <= first) {
    return {};
  }
  return std::vector<double>(samples.begin() + static_cast<ptrdiff_t>(first),
                             samples.begin() + static_cast<ptrdiff_t>(last));
}

double mean_of(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  double total = 0.0;
  for (const double value : values) {
    total += value;
  }
  return total / static_cast<double>(values.size());
}

}  // namespace

VBISampleMap make_vbi_sample_map(const VBILineLevels& levels,
                                 const VBIOutputLevels& output_levels) {
  VBISampleMap map;
  map.output_logic0 = static_cast<double>(output_levels.logic0);

  const double source_amplitude = levels.amplitude();
  if (!levels.usable || !(source_amplitude > 0.0)) {
    // Nothing gave a level reference, so there is no honest scale to apply.
    // A zero gain holds the whole record at blanking rather than amplifying
    // noise by a fabricated one (design §5.3.4).
    map.output_logic0 = static_cast<double>(output_levels.blanking);
    map.gain = 0.0;
    return map;
  }

  map.source_logic0 = levels.logic0;
  map.gain =
      static_cast<double>(output_levels.data_amplitude()) / source_amplitude;
  return map;
}

VBILevelMapperConfig vbi_absolute_level_config(
    const VBIOutputLevels& output_levels) {
  VBILevelMapperConfig config;
  config.mode = VBILevelMode::kFixed;
  config.fixed_logic0 =
      static_cast<double>(output_levels.logic0) * kTBCSampleScale;
  config.fixed_logic1 =
      static_cast<double>(output_levels.logic1) * kTBCSampleScale;
  return config;
}

VBIRecordWindows vbi_record_windows(const VBISourceFormat& format,
                                    const VBITeletextService& service,
                                    double quiet_guard_samples) {
  VBIRecordWindows windows;

  const double samples_per_bit = service.samples_per_bit(format.sample_rate_hz);
  const double cri_start = service.cri_start_samples(
      format.sample_rate_hz, format.capture_offset_samples);
  if (!(samples_per_bit > 0.0) || !(cri_start > 0.0)) {
    return windows;
  }

  const double valid_samples = static_cast<double>(format.valid_samples);
  const auto clamp_to_record = [valid_samples](double position) {
    return static_cast<uint32_t>(
        std::max(0.0, std::min(position, valid_samples)));
  };

  // Everything between the start of the record and the run-in is back porch,
  // so it is the line's own logic 0 reference.  A bt8x8 PAL record opens
  // inside the burst window, but the burst remnant is symmetric about
  // blanking and a median is unmoved by it.
  windows.quiet_begin = 0;
  windows.quiet_end =
      clamp_to_record(std::floor(cri_start - quiet_guard_samples));

  // Run-in: whole samples strictly inside the sixteen alternating bits.  A
  // service with no run-in reads its peak over the whole of its data instead,
  // whose ones are spread through it rather than gathered at its head.
  windows.cri_begin = clamp_to_record(std::ceil(cri_start));
  windows.cri_end = clamp_to_record(
      std::floor(cri_start + samples_per_bit * service.peak_reference_bits));

  // Logic 1 reference: the centre bit of the framing code's leading run of
  // ones, which is settled even on a channel blurred to a full bit period.
  if (service.frc_leading_ones >= 3) {
    const double run_start_bit = static_cast<double>(service.cri_bits);
    const double centre_bit =
        run_start_bit + static_cast<double>(service.frc_leading_ones) / 2.0;
    const double reference_begin =
        cri_start + samples_per_bit * (centre_bit - 0.5);
    const double reference_end =
        cri_start + samples_per_bit * (centre_bit + 0.5);
    windows.frc_reference_begin = clamp_to_record(std::ceil(reference_begin));
    windows.frc_reference_end = clamp_to_record(std::floor(reference_end));
    if (windows.frc_reference_end <= windows.frc_reference_begin) {
      // A source sampled at barely more than one sample per bit still needs a
      // reference sample; take the one nearest the centre of the run.
      windows.frc_reference_begin =
          clamp_to_record(std::floor(cri_start + samples_per_bit * centre_bit));
      windows.frc_reference_end = clamp_to_record(
          static_cast<double>(windows.frc_reference_begin) + 1.0);
    }
  }

  return windows;
}

VBILineLevels estimate_vbi_line_levels(const std::vector<double>& samples,
                                       const VBIRecordWindows& windows,
                                       double minimum_amplitude_counts) {
  VBILineLevels levels;

  const std::vector<double> quiet =
      window_samples(samples, windows.quiet_begin, windows.quiet_end);
  if (quiet.empty()) {
    return levels;
  }
  levels.logic0 = median_of(quiet);

  std::vector<double> cri =
      window_samples(samples, windows.cri_begin, windows.cri_end);
  double cri_logic1 = 0.0;
  if (!cri.empty()) {
    std::sort(cri.begin(), cri.end());
    cri_logic1 = sorted_quantile(cri, kCRIPeakQuantile);
  }

  const std::vector<double> frc = window_samples(
      samples, windows.frc_reference_begin, windows.frc_reference_end);
  const double frc_logic1 = frc.empty() ? 0.0 : mean_of(frc);

  // The larger of the two candidates wins.  On a clean line they agree; on a
  // band-limited one the run-in has collapsed towards the data midpoint and
  // only the framing code still reaches logic 1 (design §5.4).
  levels.logic1 = std::max(cri_logic1, frc_logic1);

  levels.usable = levels.amplitude() >= minimum_amplitude_counts;
  return levels;
}

VBILevelMapper::VBILevelMapper(const VBISourceFormat& format,
                               const VBITeletextService& service,
                               VBILevelMapperConfig config)
    : config_(config),
      windows_(
          vbi_record_windows(format, service, config.quiet_guard_samples)) {}

VBILineLevels VBILevelMapper::frame_levels(
    const std::vector<VBILineLevels>& measured) const {
  VBILineLevels levels;

  std::vector<double> logic0_values;
  std::vector<double> amplitude_values;
  logic0_values.reserve(measured.size());
  amplitude_values.reserve(measured.size());
  for (const VBILineLevels& line : measured) {
    if (!line.usable) {
      continue;
    }
    logic0_values.push_back(line.logic0);
    amplitude_values.push_back(line.amplitude());
  }
  if (logic0_values.empty()) {
    return levels;
  }

  levels.logic0 = median_of(std::move(logic0_values));
  levels.logic1 = levels.logic0 + median_of(std::move(amplitude_values));
  levels.usable = levels.amplitude() >= config_.minimum_amplitude_counts;
  return levels;
}

void VBILevelMapper::map_frame(const std::vector<VBILineRecord>& records,
                               std::vector<VBILineLevels>& out_levels) const {
  const bool fixed = config_.mode == VBILevelMode::kFixed;

  out_levels.clear();
  out_levels.reserve(records.size());
  for (const VBILineRecord& record : records) {
    out_levels.push_back(
        fixed ? VBILineLevels{}
              : estimate_vbi_line_levels(record.samples, windows_,
                                         config_.minimum_amplitude_counts));
  }

  VBILineLevels common;
  if (fixed) {
    common.logic0 = config_.fixed_logic0;
    common.logic1 = config_.fixed_logic1;
    common.usable = common.amplitude() >= config_.minimum_amplitude_counts;
  } else {
    common = frame_levels(out_levels);
  }

  const double amplitude_floor =
      common.amplitude() * config_.minimum_amplitude_fraction;

  for (VBILineLevels& levels : out_levels) {
    if (!common.usable) {
      // Nothing in the frame gave a level reference, so every line of it is
      // emitted as blanking.
      levels = VBILineLevels{};
      continue;
    }

    if (fixed) {
      levels = common;
      continue;
    }

    // A line far below the frame's amplitude carries no data service, so its
    // own estimate is noise and the frame's levels are used instead.
    bool apply_own = levels.usable && levels.amplitude() >= amplitude_floor;
    if (apply_own && config_.mode == VBILevelMode::kRolling) {
      // Hold the line at the frame's median unless it departs from it by more
      // than estimation noise, so that per-line gain noise is not propagated
      // into the output while a real gain change still is.
      const double deviation =
          std::fabs(levels.amplitude() - common.amplitude());
      apply_own =
          deviation > common.amplitude() * config_.rolling_deviation_fraction;
    }

    if (!apply_own) {
      levels = common;
    }
  }
}

}  // namespace orc
