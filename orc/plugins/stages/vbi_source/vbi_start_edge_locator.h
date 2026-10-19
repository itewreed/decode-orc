/*
 * File:        vbi_start_edge_locator.h
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Locates the first start bit of an asynchronously framed line
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VBI_START_EDGE_LOCATOR_H
#define ORC_VBI_START_EDGE_LOCATOR_H

#include <cstdint>
#include <vector>

#include "vbi_cri_correlator.h"
#include "vbi_source_format.h"
#include "vbi_teletext_service.h"

namespace orc {

// How a line has to look before its start edge is believed.
struct VBIStartEdgeConfig {
  // Smallest data swing, in source counts, that can be a data line rather than
  // noise or blanking.
  double minimum_swing_counts = 30.0;

  // Start and stop bits of the line that must read correctly.  A line of ten
  // bytes has twenty; random picture content matches fifteen or so by chance,
  // and the calibration wants positions it can trust, so the default is all of
  // them.
  uint32_t minimum_framing_bits = 20;
};

// The swing a source's word size makes meaningful: about an eighth of the
// 8-bit range a card writes, and the equivalent fraction of the absolute
// 16-bit domain a decoder writes, whose data swing is several thousand counts.
double vbi_start_edge_minimum_swing(VBISampleFormat sample_format);

// Start and stop bits of a line that read correctly, given the position of its
// first start edge and the slicing threshold.  Every byte of an asynchronously
// framed line opens with a start bit and closes with a stop bit of the
// opposite level, so a real data line scores all of them and picture content
// a little over half.  The count is taken in whichever polarity scores
// higher, so it does not depend on which level the service calls a start bit.
uint32_t vbi_line_framing_bits(const std::vector<double>& record_samples,
                               double start_edge_samples, double threshold,
                               double samples_per_bit, uint32_t bytes,
                               uint32_t bits_per_byte);

// Find the leading edge of the first start bit of a line (the anchor a
// VideoDat line's timing is measured to).
//
// The approach is the reference decoder's.  Logic 0 is read from the quiet
// samples just ahead of the search window and logic 1 as the upper decile of
// the data region; the first upward crossing of their midpoint inside the
// window is the coarse edge.  Every bit transition of the line is then folded
// onto the bit period, and the mean phase of those crossings moves the edge
// to where all of them agree — good to a small fraction of a sample on a clean
// line, which is what makes a median over a few hundred lines meaningful.
//
// The result is reported in the run-in detector's shape so the calibration can
// treat both alike: peak_correlation is the fraction of start and stop bits
// that read correctly, and a line is accepted only when it reaches the
// configured count.
VBICRIDetection detect_vbi_start_edge_position(
    const std::vector<double>& record_samples,
    const VBITeletextService& service, double sample_rate_hz,
    const VBICRISearchWindow& window, const VBIStartEdgeConfig& config);

}  // namespace orc

#endif  // ORC_VBI_START_EDGE_LOCATOR_H
