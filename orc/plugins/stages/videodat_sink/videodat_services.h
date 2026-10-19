/*
 * File:        videodat_services.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Analysis of the VideoDat services besides file transfer
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_SERVICES_H
#define ORC_VIDEODAT_SERVICES_H

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "videodat_file_assembler.h"
#include "videodat_packet.h"

namespace orc {

// Broadcast time of a byte position in the stream, from the clock packets
// (stream 01, type 06, every five seconds) and the byte rate between them.
class VideoDatClock {
 public:
  struct Tick {
    size_t position = 0;
    int64_t seconds = 0;  // VideoDatDateTime::seconds_since_epoch()
  };

  // A recording gap: the clock jumps further than the bytes between two ticks
  // allow.
  struct Gap {
    size_t position = 0;
    int64_t from = 0;
    int64_t to = 0;
    double missing_seconds = 0.0;
  };

  explicit VideoDatClock(std::vector<Tick> ticks);

  const std::vector<Tick>& ticks() const { return ticks_; }
  std::optional<double> bytes_per_second() const { return rate_; }

  // Broadcast time at a byte position, in whole seconds, or nullopt without
  // a clock.
  std::optional<VideoDatDateTime> at(size_t position) const;

  std::vector<Gap> gaps() const;

 private:
  std::vector<Tick> ticks_;
  std::optional<double> rate_;
};

struct VideoDatServiceReport {
  // The report, also among the outputs as dienste/dienste.txt.
  std::vector<std::string> lines;
  std::vector<VideoDatOutputFile> outputs;
};

// Clock, transmission schedule, active file streams, the bit lists of stream
// 00 and the raw packets of the encrypted services (account balance, credit
// limit) and of stream 01 type 05.  Output paths are under "dienste/".
VideoDatServiceReport analyse_videodat_services(
    const std::vector<VideoDatPacket>& packets);

// Run-length decoding of the bit lists: 00 n is n zero bytes, FF n is n FF
// bytes, any other byte stands for itself.
std::vector<uint8_t> videodat_unrun(const uint8_t* bytes, size_t count);

}  // namespace orc

#endif  // ORC_VIDEODAT_SERVICES_H
