/*
 * File:        vbi_line_placement.cpp
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Horizontal placement of a stored record on an output frame line
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 Simon Inns
 */

#include "vbi_line_placement.h"

#include <algorithm>
#include <cmath>

namespace orc {

namespace {

// Clamp a fractional output index onto the frame line and return it as a whole
// sample index.
uint32_t clamp_output_index(double index, uint32_t line_length) {
  if (!std::isfinite(index) || index <= 0.0) {
    return 0;
  }
  if (index >= static_cast<double>(line_length)) {
    return line_length;
  }
  return static_cast<uint32_t>(index);
}

}  // namespace

bool make_vbi_data_placement(const VBISourceFormat& format,
                             const VBITeletextService& service,
                             const VBIOutputFrame& output_frame,
                             double capture_offset_samples,
                             VBIDataPlacement& out_placement,
                             std::string& error_message) {
  out_placement = VBIDataPlacement();

  const double output_rate_hz = output_frame.sample_rate_hz;
  if (!(output_rate_hz > 0.0)) {
    error_message =
        "The output lattice has no sampling rate, so nothing can be placed on "
        "it.";
    return false;
  }

  if (!(format.sample_rate_hz > 0.0) || !std::isfinite(format.sample_rate_hz)) {
    error_message =
        "The capture's sampling rate must be a positive number of hertz.";
    return false;
  }

  if (format.valid_samples == 0) {
    error_message =
        "The source format declares no valid samples per line record, so "
        "there is nothing to place on the output line.";
    return false;
  }

  if (!std::isfinite(capture_offset_samples)) {
    error_message = "The capture offset must be a finite number of samples.";
    return false;
  }

  const double ratio = format.sample_rate_hz / output_rate_hz;
  out_placement.source_samples_per_output_sample = ratio;

  // Output sample j is at j / fs_out after 0H, and record sample n is at
  // (capture_offset + n) / fs_in after the same 0H.
  out_placement.source_position_at_output_zero = -capture_offset_samples;

  out_placement.data_start_samples =
      service.t_offset_ns * 1e-9 * output_rate_hz;

  const double transmitted_bits =
      static_cast<double>(service.transmitted_bits());
  out_placement.data_end_samples =
      (service.bit_rate_hz > 0.0)
          ? out_placement.data_start_samples +
                (transmitted_bits / service.bit_rate_hz) * output_rate_hz
          : out_placement.data_start_samples;

  // Output samples whose source coordinate falls inside the record's valid
  // samples.  Padding is already excluded upstream by the reader, so the last
  // readable coordinate is one short of the valid count.
  const double first_covered =
      std::ceil(-out_placement.source_position_at_output_zero / ratio);
  const double last_covered =
      std::floor((static_cast<double>(format.valid_samples - 1u) -
                  out_placement.source_position_at_output_zero) /
                 ratio);

  // Guard held either side of the data region: the service's guard in
  // transmitted bit periods, one for every teletext system.
  const double guard = service.samples_per_bit(output_rate_hz) *
                       static_cast<double>(service.guard_bits);
  const double region_begin =
      std::floor(out_placement.data_start_samples - guard);
  const double region_end = std::ceil(out_placement.data_end_samples + guard);

  // Every data line is at least the nominal length, and the two longer PAL
  // lines are at the foot of each field where no data service sits, so the
  // nominal length is the bound that applies to all of them.
  const uint32_t line_length = output_frame.samples_per_line_nominal;

  const uint32_t begin =
      clamp_output_index(std::max(first_covered, region_begin), line_length);
  const uint32_t end =
      clamp_output_index(std::min(last_covered + 1.0, region_end), line_length);

  out_placement.output_begin = begin;
  out_placement.output_end = std::max(begin, end);
  return true;
}

}  // namespace orc
