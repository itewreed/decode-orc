/*
 * File:        vbi_teletext_service.h
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Timing constants of the VBI data services the stage places
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 Simon Inns
 */

#ifndef ORC_VBI_TELETEXT_SERVICE_H
#define ORC_VBI_TELETEXT_SERVICE_H

#include <cstdint>
#include <string>

#include "vbi_source_format.h"

namespace orc {

// What a service's timing is measured against when the capture offset or the
// anchor is fitted.
enum class VBITimingReference {
  // A clock run-in and framing code, correlated against a generated template
  // (vbi_cri_template.h).  Every teletext system.
  kClockRunIn,

  // The leading edge of the first byte's start bit, found by the byte framing
  // around it (vbi_start_edge_locator.h).  An asynchronously framed service
  // with nothing ahead of its data, which is what VideoDat is.
  kStartBitEdge,
};

// Timing of one VBI data service, as the stage needs it to place and measure
// a data line (design §5.2).
//
// The figures follow libzvbi's service table, which is the most complete
// single reference for the 0H-to-clock-run-in anchor.
struct VBITeletextService {
  // Time from 0H to the leading edge of the first clock run-in one bit, at
  // half amplitude, in nanoseconds.
  double t_offset_ns = 0.0;

  double bit_rate_hz = 0.0;

  // Clock run-in length, in bits.  Sixteen alternating bits whose energy sits
  // at half the bit rate.
  uint32_t cri_bits = 0;

  // Framing code length, in bits.
  uint32_t frc_bits = 0;

  // Leading run of one bits at the start of the framing code.  This is the
  // shortest feature in a data line that reaches full amplitude through a
  // band-limited channel, which makes it the only trustworthy logic 1
  // reference on a blurred source (design §5.3.6).
  uint32_t frc_leading_ones = 0;

  // Combined clock run-in and framing code, in transmission order, most
  // significant bit first over cri_bits + frc_bits bits.
  uint32_t cri_frc_pattern = 0;

  // Payload length in bytes, framing code excluded.
  uint32_t payload_bytes = 0;

  // Bits transmitted on a line when the line is not a teletext packet, so that
  // run-in, framing code and payload do not describe it.  Zero for teletext.
  uint32_t line_bits = 0;

  // Bit periods of guard held either side of the data region when it is cut
  // out of a record.  One is enough for a service whose anchor is fitted to a
  // fraction of a sample; a service whose anchor varies between transmissions
  // by more than the fit can be trusted to settle needs more.
  uint32_t guard_bits = 1;

  // Bits after the anchor from which the logic 1 level is read as an upper
  // quantile.  The clock run-in for teletext; the whole data region for a
  // service with no run-in, whose ones are scattered through its data.
  uint32_t peak_reference_bits = 0;

  VBITimingReference timing_reference = VBITimingReference::kClockRunIn;

  // Bits from the anchor to the end of the last transmitted bit.
  uint32_t transmitted_bits() const {
    return (line_bits > 0) ? line_bits
                           : cri_bits + frc_bits + payload_bytes * 8u;
  }

  // Samples occupied by one transmitted bit at a source's sampling rate.
  double samples_per_bit(double sample_rate_hz) const {
    return (bit_rate_hz > 0.0) ? (sample_rate_hz / bit_rate_hz) : 0.0;
  }

  // Sample offset, within a stored line record, of the leading edge of the
  // first clock run-in one bit.
  //
  // The record's sample 0 sits capture_offset samples after 0H, so the
  // service's 0H-referred anchor moves back by exactly that much.
  double cri_start_samples(double sample_rate_hz,
                           double capture_offset_samples) const {
    return (t_offset_ns * 1e-9 * sample_rate_hz) - capture_offset_samples;
  }
};

// Timing of the data service a source carries.  Returns false with an error
// message for services the stage does not yet place.
bool vbi_teletext_service(VBITVSystem tv_system, VBITeletextSystem tt_system,
                          VBITeletextService& out_service,
                          std::string& error_message);

}  // namespace orc

#endif  // ORC_VBI_TELETEXT_SERVICE_H
