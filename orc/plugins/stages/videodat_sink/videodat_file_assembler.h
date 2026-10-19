/*
 * File:        videodat_file_assembler.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Reassembles the files carried in VideoDat packets
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_FILE_ASSEMBLER_H
#define ORC_VIDEODAT_FILE_ASSEMBLER_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "videodat_packet.h"

namespace orc {

// A calendar date and time with no time zone, as DOS and the broadcast
// clock carry them.
struct VideoDatDateTime {
  int year = 1980;
  int month = 1;
  int day = 1;
  int hour = 0;
  int minute = 0;
  int second = 0;

  // Seconds since 1970-01-01 00:00:00 of the same (unspecified) zone.
  int64_t seconds_since_epoch() const;
  static VideoDatDateTime from_seconds_since_epoch(int64_t seconds);
};

bool operator==(const VideoDatDateTime& a, const VideoDatDateTime& b);
bool operator!=(const VideoDatDateTime& a, const VideoDatDateTime& b);

// A date and time, or nullopt when a field is out of range.
std::optional<VideoDatDateTime> make_videodat_date_time(int year, int month,
                                                        int day, int hour,
                                                        int minute, int second);

// A DOS timestamp: date in the high sixteen bits, time in the low.
std::optional<VideoDatDateTime> videodat_dos_time(uint32_t value);

// "dd.mm.yyyy", "HH:MM:SS".
std::string videodat_format_date(const VideoDatDateTime& value);
std::string videodat_format_time(const VideoDatDateTime& value);

// Code page 437 text (up to the first NUL) as UTF-8.
std::string videodat_cp437_to_utf8(const uint8_t* bytes, size_t count);

// A transmitted file name made safe to write: code page 437 decoded, path and
// control characters replaced, spaces and dots trimmed; "unbenannt" when
// nothing is left.
std::string videodat_safe_name(const uint8_t* bytes, size_t count);

// One file the sink writes, its path relative to the output directory with
// '/' separators.  The modification time is the transmitted one, a local
// time.
struct VideoDatOutputFile {
  std::string path;
  std::vector<uint8_t> data;
  std::optional<VideoDatDateTime> modified;
};

// A transmitted file, complete or not.
struct VideoDatFileResult {
  uint8_t stream = 0;
  std::string name;
  uint32_t size = 0;
  size_t missing_blocks = 0;
  size_t blocks = 0;
  size_t transmissions = 0;
  // Index into VideoDatAssembly::outputs, or nullopt when not written.
  std::optional<size_t> output;
};

struct VideoDatAssemblyOptions {
  // Write incomplete files and runs of data whose header was not received,
  // missing blocks filled with zeros.
  bool keep_partial = true;
  // Collect the packets of every other service raw, by stream and type.
  bool dump_other = false;
};

struct VideoDatAssembly {
  std::vector<VideoDatOutputFile> outputs;
  std::vector<VideoDatFileResult> files;
  std::vector<std::string> log;
  size_t rescued_headers = 0;
  size_t conflicting_blocks = 0;
};

// Undo the wrap of a sub-channel's eight-bit packet counter, in reception
// order.  Small steps back are normal (every group of four packets is sent
// twice), as are steps forward (lost packets, recording pauses); a larger
// step back is a cut in the tape or a restart of the transmission and opens a
// new section that cannot overlap the previous one.  A packet whose counter
// fits neither and is not followed by packets that agree with it is an
// outlier: nullopt.
std::vector<std::optional<int64_t>> unwrap_videodat_counter(
    const std::vector<uint8_t>& counters);

// Reassemble the files of every stream.  File headers (types 02 and 0C) give
// name, size and date; data packets (type 04) follow in counter order; type
// 03 marks the end of a file.  Copies of a block received in different
// versions are decided by majority, an unrepaired packet counting double.
// Damaged headers are recovered where the counter and the file length
// confirm them.  Files sent more than once are combined.
VideoDatAssembly assemble_videodat_files(
    const std::vector<VideoDatPacket>& packets,
    const VideoDatAssemblyOptions& options);

// Header of an encrypted ShareShop file ("CHVDFILE"): the name, length and
// date of the file it holds.
struct VideoDatChvdHeader {
  std::string name;
  uint32_t length = 0;
  std::optional<VideoDatDateTime> modified;
};

std::optional<VideoDatChvdHeader> videodat_chvd_header(
    const std::vector<uint8_t>& data);

}  // namespace orc

#endif  // ORC_VIDEODAT_FILE_ASSEMBLER_H
