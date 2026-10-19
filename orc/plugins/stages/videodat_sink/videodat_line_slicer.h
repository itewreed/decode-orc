/*
 * File:        videodat_line_slicer.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Bit slicing of Pro7 Channel VideoDat lines on 4fsc PAL frames
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_LINE_SLICER_H
#define ORC_VIDEODAT_LINE_SLICER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace orc {

// The VideoDat bit layer (reconstructed from the broadcaster's receiving
// software by the reference decoder, channel-videodat-decoder): NRZ at 2
// Mbit/s, ten bytes per VBI line, each byte a start bit, eight data bits least
// significant first and a stop bit, every byte transmitted inverted.  A line
// is therefore exactly one hundred bit periods, 50 us, opening with the rising
// edge of the first start bit.
inline constexpr size_t kVideoDatBytesPerLine = 10;
inline constexpr size_t kVideoDatBitsPerByte = 10;  // start + 8 data + stop
inline constexpr size_t kVideoDatDataBitsPerByte = 8;
inline constexpr size_t kVideoDatBitsPerLine =
    kVideoDatBytesPerLine * kVideoDatBitsPerByte;
inline constexpr size_t kVideoDatDataBitsPerLine =
    kVideoDatBytesPerLine * kVideoDatDataBitsPerByte;
inline constexpr double kVideoDatBitRateHz = 2000000.0;

// Equaliser taps: samples at half-bit spacing either side of each bit centre,
// plus a constant term.
inline constexpr size_t kVideoDatEqualiserTaps = 9;
inline constexpr size_t kVideoDatFeatureColumns = kVideoDatEqualiserTaps + 1;
inline constexpr size_t kVideoDatCentreTap = kVideoDatEqualiserTaps / 2;

// Where a line's data starts and the levels it was sliced against, in 10-bit
// CVBS counts and samples from the start of the frame line.
struct VideoDatLinePhase {
  double start_edge = 0.0;
  double threshold = 0.0;
  double low = 0.0;
  double high = 0.0;
};

// Sampling of the 625-line 4 x fsc lattice the sink reads, and the windows a
// line's levels and start edge are looked for in.  The figures are the
// reference decoder's for a decoded .tbc, which is the same lattice, scaled
// from its 16-bit domain to the 10-bit one frames are carried in.
struct VideoDatSlicerConfig {
  double sample_rate_hz = 17734475.0;

  // Back porch: the logic 0 reference.  On a decoded source it holds the
  // colour burst, which is symmetric about blanking and does not move the
  // median.
  size_t quiet_begin = 100;
  size_t quiet_end = 190;

  // Where the start edge is looked for: 8,5 to 22,6 us from 0H.  Every
  // recording measured so far starts its data between 11,7 and 12,6 us.
  size_t search_begin = 150;
  size_t search_end = 400;

  // Smallest swing between the two levels that can be a data line.
  double minimum_swing = 40.0;

  // Sync tip and blanking windows for locating the end of line sync, and the
  // smallest sync amplitude that counts as a sync pulse.  A frame from the VBI
  // capture source carries no sync at all, which is how the sink tells the two
  // kinds of input apart.
  size_t sync_tip_begin = 10;
  size_t sync_tip_end = 60;
  size_t sync_blank_begin = 100;
  size_t sync_blank_end = 190;
  size_t sync_edge_begin = 60;
  size_t sync_edge_end = 130;
  double minimum_sync_amplitude = 47.0;

  double samples_per_bit() const { return sample_rate_hz / kVideoDatBitRateHz; }
};

// One sliced line: its ten bytes and, for each of their eighty data bits, how
// far the decision was from the threshold (positive meaning the line was at
// logic 1 there).  The soft values are what lets later stages combine damaged
// copies of a packet bit by bit.
struct VideoDatSlicedLine {
  std::array<uint8_t, kVideoDatBytesPerLine> bytes{};
  std::array<float, kVideoDatDataBitsPerLine> soft{};
};

// The samples around every bit centre of a line, normalised by its levels:
// one row per bit (start, data and stop bits alike), kVideoDatFeatureColumns
// columns, the last a constant 1.
using VideoDatLineFeatures =
    std::array<std::array<double, kVideoDatFeatureColumns>,
               kVideoDatBitsPerLine>;

// A linear equaliser trained on the recording's own error-free lines.  `bias`
// is added to every decision, moving the threshold in favour of logic 1: what
// errors remain on a worn tape are nearly all ones read as zeros, where the
// high level sags.
struct VideoDatEqualiser {
  std::array<double, kVideoDatFeatureColumns> weights{};
  double bias = 0.0;
};

// Start edge and levels of one line.  With a prior (the position the end of
// line sync predicts) the coarse search is skipped and only the phase is
// refined; without one the first rising edge in the search window is taken.
// Returns nullopt for a line whose swing is too small to be data.
std::optional<VideoDatLinePhase> videodat_line_phase(
    const int16_t* samples, size_t count, const VideoDatSlicerConfig& config,
    std::optional<double> prior = std::nullopt);

// Start and stop bits of a line that read correctly, 0 to 20, counted in
// whichever polarity scores higher.  A data line scores all twenty; picture
// content seldom more than fifteen.
uint32_t videodat_line_framing(const int16_t* samples, size_t count,
                               const VideoDatLinePhase& phase,
                               const VideoDatSlicerConfig& config);

// Position of the trailing edge of line sync at half amplitude, to a fraction
// of a sample, or nullopt when the line carries no sync pulse.
std::optional<double> videodat_sync_end(const int16_t* samples, size_t count,
                                        const VideoDatSlicerConfig& config);

// Feature matrix of a line.  The levels are re-estimated from the bit centres
// themselves (two-means), which follows a line whose ones sag more than its
// zeros better than the single midpoint the phase was found with.
VideoDatLineFeatures videodat_line_features(const int16_t* samples,
                                            size_t count,
                                            const VideoDatLinePhase& phase,
                                            const VideoDatSlicerConfig& config);

// Decide a line from its features: through the equaliser when one is given,
// otherwise at the centre tap alone.
VideoDatSlicedLine videodat_slice_features(
    const VideoDatLineFeatures& features,
    const VideoDatEqualiser* equaliser = nullptr, double extra_bias = 0.0);

// Bytes and soft values from one hundred bit decisions (start and stop bits
// included, which are dropped here).
VideoDatSlicedLine videodat_line_from_decisions(
    const std::array<double, kVideoDatBitsPerLine>& decisions);

// Fit an equaliser by least squares to the features of lines whose decisions
// are known to be right.  Returns nullopt when the system is singular.
std::optional<VideoDatEqualiser> videodat_fit_equaliser(
    const std::vector<const VideoDatLineFeatures*>& good_lines);

}  // namespace orc

#endif  // ORC_VIDEODAT_LINE_SLICER_H
