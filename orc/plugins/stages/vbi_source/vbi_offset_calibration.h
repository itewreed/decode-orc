/*
 * File:        vbi_offset_calibration.h
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Fits the capture offset of a card capture from its clock run-in
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 Simon Inns
 */

#ifndef ORC_VBI_OFFSET_CALIBRATION_H
#define ORC_VBI_OFFSET_CALIBRATION_H

#include <cstdint>
#include <string>
#include <vector>

#include "vbi_cri_correlator.h"
#include "vbi_cri_template.h"
#include "vbi_line_reader.h"
#include "vbi_source_format.h"
#include "vbi_start_edge_locator.h"
#include "vbi_teletext_service.h"

namespace orc {

// How tightly the fitted run-in positions agree, judged against the source
// format's own thresholds (design §5.3.4).  This is the single best health
// check the stage has.
enum class VBIOffsetSpreadClass {
  // The source is genuinely time-base corrected and the whole model holds.
  kTight,

  // Mild residual jitter or a slight sampling-rate error.  Usable, but the
  // user is told.
  kMild,

  // Either the sampling rate is wrong or the source was never time-base
  // corrected.  Not usable.
  kUnusable,
};

// One accepted run-in position.
struct VBICRIObservation {
  // Position of the record in the capture, counting every stored record of
  // every frame.  It is the abscissa a drift is fitted against, so it has to
  // be the true line ordinal rather than an index into the sampled subset.
  uint64_t line_sequence = 0;

  double anchor_position_samples = 0.0;
  double peak_correlation = 0.0;
};

// The fitted offset and everything the user needs to decide whether to trust
// it.
struct VBIOffsetCalibration {
  // False when the run must stop.  The stage does not proceed with a bad fit:
  // a wrong global offset silently mis-places every line in a multi-hour
  // decode (design §5.3.4).
  bool converged = false;

  // The fitted result: time from 0H to sample 0 of every record, in source
  // samples.  Applied globally and never per line — a per-line correction
  // would erase real timing information and shift lines that were already
  // right.
  double capture_offset_samples = 0.0;

  // Median position of the run-in within a record, from which the offset above
  // is derived.  A median rather than a mean, because a partial match on a
  // damaged line is an outlier a mean would follow.
  double anchor_position_samples = 0.0;

  // Robust spread of the accepted positions, in samples: the median absolute
  // deviation scaled to the standard deviation it would imply for a normal
  // distribution.
  double spread_samples = 0.0;

  VBIOffsetSpreadClass spread_class = VBIOffsetSpreadClass::kUnusable;

  uint64_t records_examined = 0;
  uint64_t records_accepted = 0;

  // Fraction of examined records that locked.  A diagnostic in its own right:
  // a capture with sparse teletext locks on few lines, and a configuration
  // that is wrong locks on almost none.
  double acceptance_fraction = 0.0;

  // Slope of position against line ordinal, in samples per line, and the total
  // it implies over the span examined.  A monotonic drift is diagnostic of a
  // sampling-rate error specifically.
  double drift_samples_per_line = 0.0;
  double drift_total_samples = 0.0;
  bool drift_detected = false;

  // Sampling rate the fitted drift implies, in Hz: the configured rate scaled
  // by the drift over the samples it accumulated across.  Equal to the
  // configured rate when no drift was fitted.
  double suggested_sample_rate_hz = 0.0;

  // Why the fit was rejected.  Empty when it converged.
  std::vector<std::string> diagnostics;

  // Concerns that do not stop the run.
  std::vector<std::string> warnings;

  // One-line human-readable summary of the fit.
  std::string summary;
};

// Ordinal of a record within the whole capture.
uint64_t vbi_line_sequence(const VBISourceFormat& format,
                           const VBILineRecord& record);

// Stored frames to sample for calibration, spread across the capture.
//
// Spread rather than taken from the head, so that a bad opening segment — a
// tape settling, a disc's lead-in, a mistracked first minute — cannot dominate
// a figure that is then applied to the whole file (design §5.3.4).
std::vector<uint64_t> vbi_calibration_frame_indices(uint64_t frame_count,
                                                    uint32_t sample_frames);

// Fit a global capture offset from accepted run-in positions.
//
// records_examined counts every record the search was run over, accepted or
// not, so that the acceptance fraction reflects the capture rather than the
// observations that survived.
VBIOffsetCalibration fit_vbi_capture_offset(
    const VBISourceFormat& format, const VBITeletextService& service,
    const std::vector<VBICRIObservation>& observations,
    uint64_t records_examined);

// What a calibration run should read.
struct VBICalibrationConfig {
  // Stored frames sampled across the capture.  Sixteen bt8x8 PAL frames is
  // 512 records, comfortably the few hundred lines design §5.3.4 asks for,
  // and a fraction of a second of decoding.
  uint32_t sample_frames = 16;

  VBICRITemplateConfig template_config{};
};

// Run the whole procedure against a capture: sample frames across the file,
// correlate every record of each against the run-in template, and fit the
// global offset.
//
// Returns false with an error message only when the capture could not be read
// or the configuration cannot produce a template at all.  A fit that fails its
// health checks is a converged == false result with diagnostics, not an error
// here: the difference matters because one is a broken input and the other is
// a configuration the user can correct.
bool calibrate_vbi_capture_offset(const VBILineReader& reader,
                                  const VBITeletextService& service,
                                  const VBICalibrationConfig& config,
                                  VBIOffsetCalibration& out_calibration,
                                  std::string& error_message);

// The same survey, read the other way round, for time-base corrected sources.
//
// Such a capture's offset is exactly zero and is never fitted — sample 0 of
// every record is 0H by construction (design §5.3.3) — but that leaves the
// other half of the timing unmeasured. The service's 0H-to-run-in anchor is a
// property of the transmission, not of the file, and transmissions disagree
// about it: the tabulated 525-line figures were measured on particular
// captures, and a broadcaster that transmitted its run-in a microsecond
// earlier puts it outside the window derived from them, clipping the head of
// every data line the stage places.
//
// On a card capture there is nothing to separate here, because the fitted
// capture offset already absorbs the disagreement and lands the run-in on the
// nominal anchor. On a TBC-derived one it is the anchor that has to move.
//
// The returned calibration is the survey's: capture_offset_samples is not
// meaningful for such a source and must not be applied. Read
// anchor_position_samples, or vbi_measured_anchor_ns() below.
bool measure_vbi_service_anchor(const VBILineReader& reader,
                                const VBITeletextService& service,
                                const VBICalibrationConfig& config,
                                VBIOffsetCalibration& out_calibration,
                                std::string& error_message);

// The measured anchor as a service 0H offset in nanoseconds, ready to replace
// VBITeletextService::t_offset_ns. Only meaningful for a TBC-derived source,
// whose record sample 0 is 0H.
double vbi_measured_anchor_ns(const VBIOffsetCalibration& calibration,
                              double sample_rate_hz);

}  // namespace orc

#endif  // ORC_VBI_OFFSET_CALIBRATION_H
