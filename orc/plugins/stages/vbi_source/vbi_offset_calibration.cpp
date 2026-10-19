/*
 * File:        vbi_offset_calibration.cpp
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Fits the capture offset of a card capture from its clock run-in
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 Simon Inns
 */

#include "vbi_offset_calibration.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <utility>

namespace orc {

namespace {

// Scaling that turns a median absolute deviation into the standard deviation
// it implies for a normal distribution, so that the spread can be read against
// design §5.3.4's table of standard deviations.
constexpr double kMADToStandardDeviation = 1.4826;

// Accepted positions below which a median is not a fit but an anecdote.
constexpr uint64_t kMinimumAcceptedRecords = 8;

// How many standard errors a fitted slope must exceed before it is called a
// drift rather than the estimator's own noise.  Without this, a long capture's
// enormous line span turns any residual scatter into a confident-looking
// slope.
constexpr double kDriftSignificance = 3.0;

std::string format_number(double value, int precision) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

double median_of(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2u;
  if ((values.size() % 2u) == 1u) {
    return values[middle];
  }
  return 0.5 * (values[middle - 1u] + values[middle]);
}

// What the fit locates, as its messages name it.
std::string timing_feature_name(const VBITeletextService& service) {
  return (service.timing_reference == VBITimingReference::kStartBitEdge)
             ? "start-bit edge"
             : "clock run-in";
}

// The same thing as the head of a sentence.
std::string timing_feature_sentence(const VBITeletextService& service) {
  return "The " + timing_feature_name(service);
}

std::string spread_class_name(VBIOffsetSpreadClass spread_class) {
  switch (spread_class) {
    case VBIOffsetSpreadClass::kTight:
      return "time-base corrected";
    case VBIOffsetSpreadClass::kMild:
      return "mild jitter";
    case VBIOffsetSpreadClass::kUnusable:
      return "not usable";
  }
  return "unknown";
}

}  // namespace

uint64_t vbi_line_sequence(const VBISourceFormat& format,
                           const VBILineRecord& record) {
  const uint64_t records_per_frame =
      static_cast<uint64_t>(format.field_lines) * 2u;
  return record.frame_index * records_per_frame +
         static_cast<uint64_t>(record.field_index) * format.field_lines +
         record.record_index;
}

std::vector<uint64_t> vbi_calibration_frame_indices(uint64_t frame_count,
                                                    uint32_t sample_frames) {
  std::vector<uint64_t> indices;
  if (frame_count == 0 || sample_frames == 0) {
    return indices;
  }

  const uint64_t wanted = std::min<uint64_t>(sample_frames, frame_count);
  indices.reserve(static_cast<size_t>(wanted));

  // Frames at the centres of equal divisions of the capture: the sample covers
  // the whole file, and no division's frame is the file's first or last, which
  // are the two most likely to be atypical.
  for (uint64_t index = 0; index < wanted; ++index) {
    const uint64_t frame = ((2u * index + 1u) * frame_count) / (2u * wanted);
    indices.push_back(std::min(frame, frame_count - 1u));
  }

  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  return indices;
}

VBIOffsetCalibration fit_vbi_capture_offset(
    const VBISourceFormat& format, const VBITeletextService& service,
    const std::vector<VBICRIObservation>& observations,
    uint64_t records_examined) {
  VBIOffsetCalibration calibration;
  calibration.records_examined = records_examined;
  calibration.records_accepted = observations.size();
  calibration.acceptance_fraction =
      (records_examined > 0) ? static_cast<double>(observations.size()) /
                                   static_cast<double>(records_examined)
                             : 0.0;
  calibration.suggested_sample_rate_hz = format.sample_rate_hz;

  const VBICalibrationThresholds& thresholds = format.calibration;

  if (observations.size() < kMinimumAcceptedRecords) {
    calibration.diagnostics.push_back(
        timing_feature_sentence(service) + " was found on only " +
        std::to_string(observations.size()) + " of " +
        std::to_string(records_examined) +
        " sampled records, which is too few to fit a capture offset from. "
        "Either the capture carries no data service on the configured lines, "
        "or the container format, sampling rate or data service is wrong.");
    calibration.summary = "Capture offset calibration failed: no usable " +
                          timing_feature_name(service) + ".";
    return calibration;
  }

  std::vector<double> positions;
  positions.reserve(observations.size());
  for (const VBICRIObservation& observation : observations) {
    positions.push_back(observation.anchor_position_samples);
  }

  calibration.anchor_position_samples = median_of(positions);

  std::vector<double> deviations;
  deviations.reserve(positions.size());
  for (const double position : positions) {
    deviations.push_back(
        std::abs(position - calibration.anchor_position_samples));
  }
  calibration.spread_samples =
      kMADToStandardDeviation * median_of(std::move(deviations));

  // capture_offset = t_offset - cri_position / sample_rate, in samples.
  calibration.capture_offset_samples =
      service.t_offset_ns * 1e-9 * format.sample_rate_hz -
      calibration.anchor_position_samples;

  if (calibration.spread_samples <= thresholds.tight_spread_samples) {
    calibration.spread_class = VBIOffsetSpreadClass::kTight;
  } else if (calibration.spread_samples <= thresholds.maximum_spread_samples) {
    calibration.spread_class = VBIOffsetSpreadClass::kMild;
  } else {
    calibration.spread_class = VBIOffsetSpreadClass::kUnusable;
  }

  // Least squares slope of position against line ordinal.  The ordinals are
  // taken relative to the first observation because a capture holds tens of
  // millions of lines and their absolute values would swamp the fit's
  // precision.
  const uint64_t first_sequence = observations.front().line_sequence;
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const VBICRIObservation& observation : observations) {
    sum_x += static_cast<double>(observation.line_sequence - first_sequence);
    sum_y += observation.anchor_position_samples;
  }
  const double count = static_cast<double>(observations.size());
  const double mean_x = sum_x / count;
  const double mean_y = sum_y / count;

  double covariance = 0.0;
  double variance_x = 0.0;
  double lowest_x = 0.0;
  double highest_x = 0.0;
  bool first = true;
  for (const VBICRIObservation& observation : observations) {
    const double x =
        static_cast<double>(observation.line_sequence - first_sequence);
    const double y = observation.anchor_position_samples;
    covariance += (x - mean_x) * (y - mean_y);
    variance_x += (x - mean_x) * (x - mean_x);
    if (first || x < lowest_x) {
      lowest_x = x;
    }
    if (first || x > highest_x) {
      highest_x = x;
    }
    first = false;
  }

  bool drift_significant = false;
  if (variance_x > 0.0 && observations.size() > 2u) {
    calibration.drift_samples_per_line = covariance / variance_x;
    const double intercept =
        mean_y - calibration.drift_samples_per_line * mean_x;

    double residual_energy = 0.0;
    for (const VBICRIObservation& observation : observations) {
      const double x =
          static_cast<double>(observation.line_sequence - first_sequence);
      const double predicted =
          intercept + calibration.drift_samples_per_line * x;
      const double residual = observation.anchor_position_samples - predicted;
      residual_energy += residual * residual;
    }

    const double residual_deviation =
        std::sqrt(residual_energy / (count - 2.0));
    const double slope_standard_error =
        residual_deviation / std::sqrt(variance_x);
    drift_significant = (slope_standard_error <= 0.0) ||
                        (std::abs(calibration.drift_samples_per_line) >
                         kDriftSignificance * slope_standard_error);

    calibration.drift_total_samples =
        calibration.drift_samples_per_line * (highest_x - lowest_x);
  }

  if (drift_significant && format.line_length > 0) {
    // A drift of d samples over N lines means the capture actually holds
    // d more samples than N x line_length of them, so the true rate is the
    // configured one scaled by that fraction (design §5.3.4).
    calibration.suggested_sample_rate_hz =
        format.sample_rate_hz *
        (1.0 + calibration.drift_samples_per_line /
                   static_cast<double>(format.line_length));
  }

  calibration.drift_detected =
      drift_significant && (std::abs(calibration.drift_total_samples) >
                            thresholds.maximum_drift_samples);

  if (calibration.acceptance_fraction <
      thresholds.minimum_acceptance_fraction) {
    calibration.diagnostics.push_back(
        timing_feature_sentence(service) + " was found on only " +
        format_number(calibration.acceptance_fraction * 100.0, 1) +
        "% of the sampled records, below the " +
        format_number(thresholds.minimum_acceptance_fraction * 100.0, 1) +
        "% this source format expects. The capture may carry its data on "
        "different lines than configured, or the container format may be "
        "wrong.");
  }

  if (calibration.spread_class == VBIOffsetSpreadClass::kUnusable) {
    calibration.diagnostics.push_back(
        timing_feature_sentence(service) + " position varies by " +
        format_number(calibration.spread_samples, 2) +
        " samples across the capture, above this format's limit of " +
        format_number(thresholds.maximum_spread_samples, 2) +
        " samples. Either the configured sampling rate of " +
        format_number(format.sample_rate_hz, 0) +
        " Hz is wrong, or the source is not time-base corrected. A single "
        "global capture offset cannot describe it.");
  } else if (calibration.spread_class == VBIOffsetSpreadClass::kMild) {
    calibration.warnings.push_back(
        timing_feature_sentence(service) + " position varies by " +
        format_number(calibration.spread_samples, 2) + " samples, above the " +
        format_number(thresholds.tight_spread_samples, 2) +
        " samples a cleanly time-base corrected source shows. This is "
        "residual jitter or a slight sampling-rate error; the fit is usable "
        "but worth checking.");
  }

  if (calibration.drift_detected) {
    calibration.diagnostics.push_back(
        timing_feature_sentence(service) +
        " position drifts monotonically by " +
        format_number(calibration.drift_total_samples, 2) +
        " samples across the sampled span, which means the configured "
        "sampling rate of " +
        format_number(format.sample_rate_hz, 0) +
        " Hz is wrong. The drift implies a true rate of " +
        format_number(calibration.suggested_sample_rate_hz, 0) +
        " Hz. A global capture offset cannot correct a rate error.");
  }

  calibration.converged = calibration.diagnostics.empty();

  calibration.summary =
      std::string(calibration.converged ? "Capture offset "
                                        : "Capture offset (rejected) ") +
      format_number(calibration.capture_offset_samples, 2) + " samples, " +
      (service.timing_reference == VBITimingReference::kStartBitEdge
           ? "start edge at "
           : "run-in at ") +
      format_number(calibration.anchor_position_samples, 2) +
      " samples, spread " + format_number(calibration.spread_samples, 2) +
      " samples (" + spread_class_name(calibration.spread_class) +
      "), locked on " + std::to_string(calibration.records_accepted) + " of " +
      std::to_string(calibration.records_examined) + " records (" +
      format_number(calibration.acceptance_fraction * 100.0, 1) + "%).";

  return calibration;
}

namespace {

// The measurement both entry points below are built on: correlate the run-in
// template against every data record of a sample of frames and fit the result.
//
// The survey is one procedure because the question it answers is one question —
// where is the clock run-in? — and only what the answer is then used for
// differs between the two families (design §5.3.3).
bool survey_vbi_cri_positions(const VBILineReader& reader,
                              const VBITeletextService& service,
                              const VBICalibrationConfig& config,
                              VBIOffsetCalibration& out_calibration,
                              std::string& error_message) {
  const VBISourceFormat& format = reader.format();

  // A service with a clock run-in is found by correlating a template of it; one
  // without is found by the edge its first byte opens with, so it has no
  // template to build.
  const bool start_bit_edge =
      service.timing_reference == VBITimingReference::kStartBitEdge;

  VBICRITemplate cri_template;
  if (!start_bit_edge &&
      !make_vbi_cri_frc_template(service, format.sample_rate_hz,
                                 config.template_config, cri_template,
                                 error_message)) {
    return false;
  }

  VBIStartEdgeConfig edge_config;
  edge_config.minimum_swing_counts =
      vbi_start_edge_minimum_swing(format.sample_format);

  std::vector<uint64_t> frames;
  const std::optional<uint64_t> frame_count = reader.frame_count();
  if (frame_count.has_value()) {
    frames = vbi_calibration_frame_indices(*frame_count, config.sample_frames);
  } else {
    // A transport that cannot report its length can still be calibrated, but
    // only from its head; the sample is no longer spread across the capture
    // and the user is told so.
    for (uint32_t index = 0; index < config.sample_frames; ++index) {
      frames.push_back(index);
    }
  }
  if (frames.empty()) {
    error_message = "The capture holds no whole frames to calibrate from.";
    return false;
  }

  const VBICRISearchWindow window = vbi_cri_search_window(format, service);

  std::vector<VBICRIObservation> observations;
  uint64_t records_examined = 0;
  for (const uint64_t frame : frames) {
    VBIFrameRecords records;
    if (!reader.read_frame(frame, records, error_message)) {
      return false;
    }

    for (const VBILineRecord& record : records.lines) {
      ++records_examined;
      const VBICRIDetection detection =
          start_bit_edge ? detect_vbi_start_edge_position(
                               record.samples, service, format.sample_rate_hz,
                               window, edge_config)
                         : detect_vbi_cri_position(
                               record.samples, cri_template, window,
                               format.calibration.acceptance_correlation);
      if (!detection.accepted) {
        continue;
      }

      VBICRIObservation observation;
      observation.line_sequence = vbi_line_sequence(format, record);
      observation.anchor_position_samples = detection.anchor_position_samples;
      observation.peak_correlation = detection.peak_correlation;
      observations.push_back(observation);
    }
  }

  out_calibration =
      fit_vbi_capture_offset(format, service, observations, records_examined);

  if (!frame_count.has_value()) {
    out_calibration.warnings.push_back(
        "The transport could not report the capture's length, so the "
        "calibration sample was taken from the head of the file rather than "
        "spread across it.");
  }
  return true;
}

}  // namespace

bool calibrate_vbi_capture_offset(const VBILineReader& reader,
                                  const VBITeletextService& service,
                                  const VBICalibrationConfig& config,
                                  VBIOffsetCalibration& out_calibration,
                                  std::string& error_message) {
  out_calibration = VBIOffsetCalibration();

  if (reader.format().family == VBISourceFamily::kTBCDerived) {
    error_message =
        "A time-base corrected source has sample 0 of every record at 0H by "
        "construction, so its capture offset is exactly zero and must not be "
        "calibrated (design §5.3.3).";
    return false;
  }

  return survey_vbi_cri_positions(reader, service, config, out_calibration,
                                  error_message);
}

bool measure_vbi_service_anchor(const VBILineReader& reader,
                                const VBITeletextService& service,
                                const VBICalibrationConfig& config,
                                VBIOffsetCalibration& out_calibration,
                                std::string& error_message) {
  out_calibration = VBIOffsetCalibration();

  if (reader.format().family != VBISourceFamily::kTBCDerived) {
    error_message =
        "The service anchor is only measured on a time-base corrected source. "
        "On a card capture the anchor is not separable from the capture "
        "offset — the fitted offset already puts the run-in at the service's "
        "nominal time — so measuring it would be fitting one unknown twice.";
    return false;
  }

  return survey_vbi_cri_positions(reader, service, config, out_calibration,
                                  error_message);
}

double vbi_measured_anchor_ns(const VBIOffsetCalibration& calibration,
                              double sample_rate_hz) {
  if (!(sample_rate_hz > 0.0)) {
    return 0.0;
  }
  // Sample 0 of the record is 0H, so where the run-in sits within the record
  // is, without any further correction, how long after 0H it was transmitted.
  return calibration.anchor_position_samples / sample_rate_hz * 1e9;
}

}  // namespace orc
