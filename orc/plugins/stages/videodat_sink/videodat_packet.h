/*
 * File:        videodat_packet.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     VideoDat packet framing, descrambling and multi-copy repair
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_PACKET_H
#define ORC_VIDEODAT_PACKET_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "videodat_line_slicer.h"

namespace orc {

// The byte stream recovered from the data lines, in transmission order, with
// one soft value per data bit.  Soft values are quantised to a signed byte —
// the stream of a four-hour recording would otherwise run to most of a
// gigabyte — preserving the sign of every decision exactly.
struct VideoDatByteStream {
  std::vector<uint8_t> data;
  std::vector<int8_t> soft;

  void append(const VideoDatSlicedLine& line);
};

// Scale of the soft quantisation: a decision one level swing from the
// threshold is 64.
inline constexpr double kVideoDatSoftScale = 64.0;

int8_t videodat_quantise_soft(double soft);

// How a packet came to pass its checksum.
enum class VideoDatRecovery {
  kNone,       // it has not
  kReceived,   // as received
  kUnstuffed,  // after removing the escape bytes a sync word in it was sent
               // with
  kSoft,       // by soft decision over several received copies
  kCopies,     // by combining bytes or spans of several received copies
};

// One packet as found in the stream (reference decoder, videodat_decoder.py
// header comment):
//
//   0   56 AE    sync
//   2   A        data stream / service
//   3   B        bits 0-3: n, payload is 2^n bytes; bit 7: sub-channel
//   4   C        packet counter, per stream
//   5-7 D E F    permutation of the 8-byte payload blocks
//   8   S        key: k = S xor 0x58
//   9   payload  2^n bytes, scrambled
//   9+2^n        checksum lo/hi = (A + B + C + sum(clear payload)) & 0xFFFF
struct VideoDatPacket {
  size_t position = 0;  // byte offset of the sync word in the stream
  std::vector<uint8_t> raw;
  std::vector<uint8_t> payload;
  std::vector<int8_t> soft;  // eight per raw byte, or empty
  bool ok = false;
  VideoDatRecovery recovery = VideoDatRecovery::kNone;

  uint8_t stream() const { return raw.size() > 2 ? raw[2] : 0; }
  uint8_t control() const { return raw.size() > 3 ? raw[3] : 0; }
  uint8_t counter() const { return raw.size() > 4 ? raw[4] : 0; }
  uint8_t sub_channel() const { return static_cast<uint8_t>(control() >> 7); }

  // True for a packet received as it is, without any repair: such a packet
  // counts double when copies of a file block disagree.
  bool unrepaired() const {
    return recovery == VideoDatRecovery::kReceived ||
           recovery == VideoDatRecovery::kUnstuffed;
  }
};

// Length of the packet whose header starts at raw[0], from its size field.
size_t videodat_packet_length(uint8_t control);

// Descramble a whole packet into its payload.  Returns false when the
// checksum fails or the packet is shorter than its size field says; the
// payload is still returned in the first case, so a damaged header can be
// looked at.
bool videodat_descramble(const std::vector<uint8_t>& raw,
                         std::vector<uint8_t>& out_payload);

// The checksum without descrambling: the permutation does not change a sum,
// so the raw payload bytes xor the key add up to the same figure.
bool videodat_checksum_ok(const std::vector<uint8_t>& raw);

// Every packet in the stream.  A packet whose checksum fails is tried again
// with the escape byte removed after any sync word inside it.
std::vector<VideoDatPacket> find_videodat_packets(
    const VideoDatByteStream& stream);
std::vector<VideoDatPacket> find_videodat_packets(
    const std::vector<uint8_t>& data);

// Counts of the packets repair_videodat_packets() fixed, by method.  A group
// of copies fixed together counts once.
struct VideoDatRepairCounts {
  size_t soft = 0;
  size_t copies = 0;
};

// Repair damaged packets from the other copies of them in the stream.  Every
// packet is sent twice and many repeatedly, and copies are recognised by their
// identical header.  Methods, in order:
//
//   1. Add the soft values of all copies and decide again.  With two or more
//      copies, then try flipping the bits the copies disagree on, accepting
//      only a unique combination that passes the checksum.  A single copy is
//      never repaired on suspicion: the checksum is a plain sum and cannot say
//      where an error is.
//   2. Combine whole bytes where copies differ in at most fourteen positions
//      and only two values each, accepting a unique passing combination.
//   3. Take the head of one copy and the tail of another, or one copy with a
//      lost frame's sixty bytes filled in from another.
//
// Every copy of the group receives the repaired packet.
VideoDatRepairCounts repair_videodat_packets(
    std::vector<VideoDatPacket>& packets);

}  // namespace orc

#endif  // ORC_VIDEODAT_PACKET_H
