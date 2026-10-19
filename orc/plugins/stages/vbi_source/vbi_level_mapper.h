/*
 * File:        vbi_level_mapper.h
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Maps card-capture sample levels into the CVBS amplitude domain
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 Simon Inns
 */

#ifndef ORC_VBI_LEVEL_MAPPER_H
#define ORC_VBI_LEVEL_MAPPER_H

#include <cstdint>
#include <vector>

#include "vbi_line_reader.h"
#include "vbi_output_frame.h"
#include "vbi_source_format.h"
#include "vbi_teletext_service.h"

namespace orc {

// How the mapper decides which levels to apply to a line.
enum class VBILevelMode {
  // Each line is normalised by its own estimate.  Follows fast gain changes
  // but carries per-line estimation noise into the output.
  kPerLine,

  // Levels are the median over the lines of a stored frame, with a line's own
  // estimate used only where it deviates significantly from that median.
  kRolling,

  // Configured levels applied to every line; nothing is estimated.
  kFixed,
};

// Level estimation and mapping policy.
struct VBILevelMapperConfig {
  VBILevelMode mode = VBILevelMode::kPerLine;

  // Margin held clear of the nominal clock run-in when reading the quiet
  // region, in source samples.  Three samples is about 85 ns at 8 x fsc and
  // gives the bt8x8 PAL back-porch window of samples 0-117 in design §5.4.
  double quiet_guard_samples = 3.0;

  // Smallest data amplitude, in source counts, that can be a measurement
  // rather than noise.  A line below it carries no data service, so its own
  // estimate is discarded.
  double minimum_amplitude_counts = 8.0;

  // A line whose amplitude falls below this fraction of the frame's median
  // amplitude is likewise treated as carrying no data service.
  double minimum_amplitude_fraction = 0.25;

  // Relative departure from the frame's median amplitude at which kRolling
  // stops holding a line at the median and applies the line's own estimate.
  double rolling_deviation_fraction = 0.25;

  // Source-domain levels used by kFixed.
  double fixed_logic0 = 0.0;
  double fixed_logic1 = 0.0;
};

// The factor between the 16-bit domain ld-decode and vhs-decode write and the
// 10-bit CVBS_U10_4FSC domain the stage emits.  Both are anchored to the same
// physical levels — PAL blanking is 256 in one and 16 384 in the other, NTSC
// white 800 and 51 200 — so the whole mapping is this one scale.
constexpr double kTBCSampleScale = 64.0;

// Levels for a source whose samples are already in the output's amplitude
// domain, scaled by the factor above.
//
// A time-base corrected capture has absolute levels: the decoder that produced
// it has already normalised blanking and white to fixed values, so estimating
// them again from the record's own structure would replace a known quantity
// with a measured one.  It would also be actively wrong on these captures,
// because a .tbc record starts at 0H and so opens with the line's sync pulse
// and colour burst, neither of which is the blanking the quiet-region estimate
// assumes it is reading.
//
// Expressed as fixed levels rather than as a mode of its own, because that is
// exactly what it is: the two logic levels are known, so nothing is measured.
// The pair chosen does not matter as long as both come from the same domain —
// any pair gives the same affine map, which is a division by the scale.
VBILevelMapperConfig vbi_absolute_level_config(
    const VBIOutputLevels& output_levels);

// Sample windows within a stored line record from which levels are read.
// Ranges are half-open, in samples from the start of the record.
struct VBIRecordWindows {
  // Quiet region between the start of the record and the clock run-in: the
  // logic 0 reference.
  uint32_t quiet_begin = 0;
  uint32_t quiet_end = 0;

  // The clock run-in, or for a service without one the data region: the
  // window whose upper quantile is the logic 1 reference.
  uint32_t cri_begin = 0;
  uint32_t cri_end = 0;

  // Centre bit of the framing code's leading run of ones: the logic 1
  // reference that survives a band-limited channel.
  uint32_t frc_reference_begin = 0;
  uint32_t frc_reference_end = 0;
};

// The levels applied to one line record, in source-domain counts.
struct VBILineLevels {
  double logic0 = 0.0;
  double logic1 = 0.0;

  // False when no level reference could be established, in which case the line
  // carries no data service and is emitted as blanking.
  bool usable = false;

  double amplitude() const { return logic1 - logic0; }
};

// The affine map from a record's own amplitude domain into the output's.
//
// Applied after resampling rather than before it.  The two are equivalent for a
// filter whose response at DC is exactly one — which is what the resampler's
// per-sample weight normalisation guarantees — and mapping the far smaller
// output saves a whole pass over every record.
struct VBISampleMap {
  double source_logic0 = 0.0;
  double output_logic0 = 0.0;
  double gain = 0.0;

  double apply(double sample) const {
    return output_logic0 + (sample - source_logic0) * gain;
  }
};

// Build the map from a line's measured levels onto the output's.  An unusable
// or degenerate measurement yields a map that produces blanking, so a line with
// no honest scale is never amplified by a fabricated one.
VBISampleMap make_vbi_sample_map(const VBILineLevels& levels,
                                 const VBIOutputLevels& output_levels);

// Derive the level-reading windows of a source format.
VBIRecordWindows vbi_record_windows(const VBISourceFormat& format,
                                    const VBITeletextService& service,
                                    double quiet_guard_samples);

// Read the levels of one line record.
VBILineLevels estimate_vbi_line_levels(const std::vector<double>& samples,
                                       const VBIRecordWindows& windows,
                                       double minimum_amplitude_counts);

// Establishes the amplitude domain of u8 card captures — the sources whose
// levels are relative and affected by automatic gain control.
//
// Levels are estimated per line from the record's own structure: logic 0 from
// the quiet region ahead of the clock run-in, and logic 1 from the larger of
// the run-in's peaks and the framing code's leading run of ones.  Taking the
// larger is what stops a band-limited source under-scaling: on the VHS waveform
// measured in design §5.3.6 the run-in has collapsed to a few per cent of the
// data amplitude, and normalising to it would produce output at a fraction of
// the correct amplitude.
//
// The mapping the levels imply is linear and nothing else is done to the
// samples.  A deconvolving slicer downstream recovers data by matching the
// blurred waveform it is given, so any sharpening, slicing or re-quantisation
// here would destroy the information it depends on (design §5.3.6).
//
// The mapper works on a whole stored frame at a time.  That is what makes
// kRolling's median deterministic: frames are built lazily and in whatever
// order they are asked for, so a window carried across frames would make a
// frame's output depend on which frames were read before it.
//
// Thread-compatible: instances hold configuration only.
class VBILevelMapper {
 public:
  VBILevelMapper() = default;

  VBILevelMapper(const VBISourceFormat& format,
                 const VBITeletextService& service,
                 VBILevelMapperConfig config);

  const VBIRecordWindows& windows() const { return windows_; }

  const VBILevelMapperConfig& config() const { return config_; }

  // Levels to apply to every record of one stored frame.  out_levels holds one
  // entry per input record, in the same order.
  void map_frame(const std::vector<VBILineRecord>& records,
                 std::vector<VBILineLevels>& out_levels) const;

 private:
  // Levels held in common by a frame, from which lines are mapped when their
  // own measurement is not used.
  VBILineLevels frame_levels(const std::vector<VBILineLevels>& measured) const;

  VBILevelMapperConfig config_{};
  VBIRecordWindows windows_{};
};

}  // namespace orc

#endif  // ORC_VBI_LEVEL_MAPPER_H
