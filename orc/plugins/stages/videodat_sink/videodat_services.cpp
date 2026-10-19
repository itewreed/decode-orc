/*
 * File:        videodat_services.cpp
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Analysis of the VideoDat services besides file transfer
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_services.h"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <tuple>

namespace orc {

namespace {

// Services as reconstructed by the reference decoder from a recording of
// 11 April 1993:
//
//   stream 01 type 06       clock: 06 YY MM DD hh mm ss 00, every 5 seconds
//   stream 01 type 05       64 15-bit pseudo-random values (purpose unknown)
//   stream 03 types 07/08   account balance, VideoDat DES with 8 rounds
//   stream 03 type 0A       change of credit limit, the same cipher
//   stream 00 type 00       bit lists in 32-byte blocks, run-length coded:
//                           00 [list] [block LE16] 00 [count LE16] data
//   stream 00 types 01/02/81/82  control: [type][stream]; 82/81 start/end of
//                           a transmission on sub-channel 1, 02/01 likewise
//                           on sub-channel 0 (meaning less certain)
//   on sub-channel 1, type 00  transmission header: 00 [DOS time 4] [2]
//                           [length] [name]; type 01 its end: 01 [DOS time 4]
constexpr uint8_t kClockStream = 0x01;
constexpr uint8_t kClockType = 0x06;
constexpr size_t kClockLength = 8;

// A clock tick agrees with a neighbour within this many seconds.
constexpr int64_t kClockAgreementSeconds = 900;
// Neighbours on each side a tick is compared with.
constexpr size_t kClockNeighbours = 3;
// Ticks further apart than this do not measure the byte rate.
constexpr int64_t kClockRateMaximumSeconds = 30;
// A clock that runs ahead of the bytes by more than this marks a gap.
constexpr double kClockGapSeconds = 30.0;

constexpr size_t kBitListBlock = 32;
constexpr size_t kBitListMinimumPayload = 16;

// Streams with fewer file-transfer packets than this are not listed.
constexpr size_t kMinimumActivePackets = 3;

// A list counts as a subset of another when at most this fraction of its set
// bits lies outside it.
constexpr double kSubsetTolerance = 0.1;

std::string hex2(unsigned value) { return fmt::format("{:02X}", value); }

std::string time_or_unknown(const std::optional<VideoDatDateTime>& value) {
  return value.has_value() ? videodat_format_time(*value) : "?";
}

std::optional<VideoDatDateTime> dos_date_time(const std::vector<uint8_t>& p,
                                              size_t offset) {
  const uint32_t time = static_cast<uint32_t>(p[offset] | p[offset + 1] << 8);
  const uint32_t date =
      static_cast<uint32_t>(p[offset + 2] | p[offset + 3] << 8);
  return videodat_dos_time(date << 16 | time);
}

double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const size_t n = values.size();
  return (n % 2 == 1) ? values[n / 2]
                      : 0.5 * (values[n / 2 - 1] + values[n / 2]);
}

// Most frequent value, the first seen among equals.
const std::vector<uint8_t>& most_common(
    const std::vector<std::pair<std::vector<uint8_t>, size_t>>& counts) {
  const auto* best = &counts.front();
  for (const auto& entry : counts) {
    if (entry.second > best->second) best = &entry;
  }
  return best->first;
}

std::vector<uint8_t> unpack_bits(const std::vector<uint8_t>& bytes) {
  std::vector<uint8_t> bits;
  bits.reserve(bytes.size() * 8);
  for (const uint8_t byte : bytes) {
    for (int bit = 7; bit >= 0; --bit) bits.push_back((byte >> bit) & 1u);
  }
  return bits;
}

}  // namespace

VideoDatClock::VideoDatClock(std::vector<Tick> ticks)
    : ticks_(std::move(ticks)) {
  std::sort(ticks_.begin(), ticks_.end(), [](const Tick& a, const Tick& b) {
    return std::tie(a.position, a.seconds) < std::tie(b.position, b.seconds);
  });
  std::vector<double> rates;
  for (size_t i = 1; i < ticks_.size(); ++i) {
    const int64_t dt = ticks_[i].seconds - ticks_[i - 1].seconds;
    if (dt > 0 && dt <= kClockRateMaximumSeconds) {
      rates.push_back(
          static_cast<double>(ticks_[i].position - ticks_[i - 1].position) /
          static_cast<double>(dt));
    }
  }
  if (!rates.empty()) rate_ = median(rates);
}

std::optional<VideoDatDateTime> VideoDatClock::at(size_t position) const {
  if (ticks_.empty() || !rate_.has_value() || !(*rate_ != 0.0)) {
    return std::nullopt;
  }
  const auto next =
      std::lower_bound(ticks_.begin(), ticks_.end(), position,
                       [](const Tick& t, size_t p) { return t.position < p; });
  const Tick* best = nullptr;
  const auto distance = [&](const Tick& t) {
    return std::abs(static_cast<double>(t.position) -
                    static_cast<double>(position));
  };
  if (next != ticks_.begin()) best = &*(next - 1);
  if (next != ticks_.end() &&
      (best == nullptr || distance(*next) < distance(*best))) {
    best = &*next;
  }
  const double offset =
      (static_cast<double>(position) - static_cast<double>(best->position)) /
      *rate_;
  return VideoDatDateTime::from_seconds_since_epoch(
      best->seconds + static_cast<int64_t>(std::floor(offset)));
}

std::vector<VideoDatClock::Gap> VideoDatClock::gaps() const {
  std::vector<Gap> out;
  if (!rate_.has_value()) return out;
  for (size_t i = 1; i < ticks_.size(); ++i) {
    const double dt =
        static_cast<double>(ticks_[i].seconds - ticks_[i - 1].seconds);
    const double expected =
        static_cast<double>(ticks_[i].position - ticks_[i - 1].position) /
        *rate_;
    if (dt - expected > kClockGapSeconds || dt < 0.0) {
      out.push_back({ticks_[i - 1].position, ticks_[i - 1].seconds,
                     ticks_[i].seconds, dt - expected});
    }
  }
  return out;
}

std::vector<uint8_t> videodat_unrun(const uint8_t* bytes, size_t count) {
  std::vector<uint8_t> out;
  size_t i = 0;
  while (i < count) {
    if (bytes[i] == 0x00 || bytes[i] == 0xFF) {
      if (i + 1 >= count) break;
      out.insert(out.end(), bytes[i + 1], bytes[i]);
      i += 2;
    } else {
      out.push_back(bytes[i]);
      ++i;
    }
  }
  return out;
}

VideoDatServiceReport analyse_videodat_services(
    const std::vector<VideoDatPacket>& packets) {
  VideoDatServiceReport report;
  std::vector<std::string>& w = report.lines;

  std::vector<const VideoDatPacket*> ok;
  for (const VideoDatPacket& packet : packets) {
    if (packet.ok && !packet.payload.empty()) ok.push_back(&packet);
  }

  // --- Clock ---------------------------------------------------------------
  std::vector<VideoDatClock::Tick> ticks;
  for (const VideoDatPacket* x : ok) {
    const auto& p = x->payload;
    if (x->stream() == kClockStream && p[0] == kClockType &&
        p.size() == kClockLength) {
      const auto t =
          make_videodat_date_time(1900 + p[1], p[2], p[3], p[4], p[5], p[6]);
      if (t.has_value()) {
        ticks.push_back({x->position, t->seconds_since_epoch()});
      }
    }
  }
  std::sort(ticks.begin(), ticks.end(),
            [](const VideoDatClock::Tick& a, const VideoDatClock::Tick& b) {
              return std::tie(a.position, a.seconds) <
                     std::tie(b.position, b.seconds);
            });
  // Drop outliers (a wrong checksum that happened to pass).
  std::vector<VideoDatClock::Tick> plausible;
  for (size_t k = 0; k < ticks.size(); ++k) {
    const size_t first = k >= kClockNeighbours ? k - kClockNeighbours : 0;
    const size_t last = std::min(ticks.size(), k + kClockNeighbours + 1);
    size_t neighbours = 0;
    size_t agreeing = 0;
    for (size_t j = first; j < last; ++j) {
      if (j == k) continue;
      ++neighbours;
      if (std::llabs(ticks[k].seconds - ticks[j].seconds) <
          kClockAgreementSeconds) {
        ++agreeing;
      }
    }
    if (agreeing >= std::min<size_t>(2, neighbours)) {
      plausible.push_back(ticks[k]);
    }
  }
  const VideoDatClock clock(plausible);
  w.push_back("CLOCK (stream 01, type 06)");
  if (!plausible.empty()) {
    const auto first =
        VideoDatDateTime::from_seconds_since_epoch(plausible.front().seconds);
    const auto last =
        VideoDatDateTime::from_seconds_since_epoch(plausible.back().seconds);
    w.push_back(fmt::format(
        "  {} timestamps, {} {} to {}{}", plausible.size(),
        videodat_format_date(first), videodat_format_time(first),
        videodat_format_time(last),
        clock.bytes_per_second().has_value()
            ? fmt::format(", {:.0f} bytes/s", *clock.bytes_per_second())
            : std::string()));
    for (const auto& gap : clock.gaps()) {
      w.push_back(fmt::format(
          "  recording gap at byte {}: {} -> {} (about {} s missing)",
          gap.position,
          videodat_format_time(
              VideoDatDateTime::from_seconds_since_epoch(gap.from)),
          videodat_format_time(
              VideoDatDateTime::from_seconds_since_epoch(gap.to)),
          static_cast<int64_t>(gap.missing_seconds)));
    }
  } else {
    w.push_back("  no clock packets found");
  }

  // --- Transmission schedule ----------------------------------------------
  w.push_back("");
  w.push_back("TRANSMISSION SCHEDULE");
  std::vector<std::pair<size_t, std::string>> events;
  for (const VideoDatPacket* x : ok) {
    const auto& p = x->payload;
    const bool sub_channel_1 = (x->control() & 0x80) != 0;
    if (x->stream() == 0x00 &&
        (p[0] == 0x01 || p[0] == 0x02 || p[0] == 0x81 || p[0] == 0x82) &&
        p.size() == 8) {
      const char* kind = p[0] == 0x82   ? "start"
                         : p[0] == 0x81 ? "end"
                         : p[0] == 0x02 ? "start?"
                                        : "end?";
      events.emplace_back(x->position, fmt::format("Control: {:<7} stream {}",
                                                   kind, hex2(p[1])));
    } else if (sub_channel_1 && p[0] == 0x00 && p.size() > 9) {
      const size_t end = std::min<size_t>(8 + p[7], p.size());
      std::string name = videodat_cp437_to_utf8(p.data() + 8, end - 8);
      const size_t a = name.find_first_not_of(" \t\r\n\v\f");
      const size_t b = name.find_last_not_of(" \t\r\n\v\f");
      name =
          (a == std::string::npos) ? std::string() : name.substr(a, b - a + 1);
      events.emplace_back(
          x->position,
          fmt::format("Stream {} transmission header '{}', air time {}",
                      hex2(x->stream()), name,
                      time_or_unknown(dos_date_time(p, 1))));
    } else if (sub_channel_1 && p[0] == 0x01 && p.size() == 8) {
      events.emplace_back(
          x->position,
          fmt::format("Stream {} transmission end, time {}", hex2(x->stream()),
                      time_or_unknown(dos_date_time(p, 1))));
    }
  }
  std::sort(events.begin(), events.end());
  const std::string* previous = nullptr;
  for (const auto& [position, text] : events) {
    if (previous != nullptr && *previous == text) continue;  // a repeat
    previous = &text;
    w.push_back(
        fmt::format("  {}  {}", time_or_unknown(clock.at(position)), text));
  }
  w.push_back("");
  w.push_back("  Active data streams (file transfer):");
  std::map<std::pair<uint8_t, uint8_t>, std::vector<size_t>> active;
  for (const VideoDatPacket* x : ok) {
    const uint8_t t = x->payload[0];
    if (x->stream() != 0x00 &&
        (t == 0x02 || t == 0x03 || t == 0x04 || t == 0x0C)) {
      active[{x->stream(), x->sub_channel()}].push_back(x->position);
    }
  }
  for (const auto& [key, positions] : active) {
    if (positions.size() < kMinimumActivePackets) continue;
    const auto [lo, hi] =
        std::minmax_element(positions.begin(), positions.end());
    w.push_back(fmt::format("    Stream {}/{}: {} - {}, {} packets",
                            hex2(key.first), key.second,
                            time_or_unknown(clock.at(*lo)),
                            time_or_unknown(clock.at(*hi)), positions.size()));
  }

  // --- Bit lists ----------------------------------------------------------
  using Counts = std::vector<std::pair<std::vector<uint8_t>, size_t>>;
  std::map<uint8_t, std::map<size_t, Counts>> lists;
  for (const VideoDatPacket* x : ok) {
    const auto& p = x->payload;
    if (x->stream() != 0x00 || p[0] != 0x00 ||
        p.size() < kBitListMinimumPayload) {
      continue;
    }
    const size_t block = static_cast<size_t>(p[2] | p[3] << 8);
    const size_t count = static_cast<size_t>(p[5] | p[6] << 8);
    const std::vector<uint8_t> data =
        videodat_unrun(p.data() + 7, p.size() - 7);
    for (size_t k = 0; k < count; ++k) {
      if ((k + 1) * kBitListBlock > data.size()) continue;
      std::vector<uint8_t> segment(
          data.begin() + static_cast<ptrdiff_t>(k * kBitListBlock),
          data.begin() + static_cast<ptrdiff_t>((k + 1) * kBitListBlock));
      Counts& counts = lists[p[1]][block + k];
      auto it = std::find_if(counts.begin(), counts.end(),
                             [&](const auto& c) { return c.first == segment; });
      if (it == counts.end()) {
        counts.emplace_back(std::move(segment), 1);
      } else {
        ++it->second;
      }
    }
  }
  w.push_back("");
  w.push_back(
      "BIT LISTS (stream 00, type 00) - probably enabling per service, one "
      "bit");
  w.push_back(
      "per decoder/subscriber number (bit 7 of the first byte = number 0)");
  std::map<uint8_t, std::vector<uint8_t>> bits;
  for (const auto& [list, blocks] : lists) {
    const size_t height = blocks.rbegin()->first + 1;
    std::vector<uint8_t> data;
    data.reserve(height * kBitListBlock);
    for (size_t y = 0; y < height; ++y) {
      const auto it = blocks.find(y);
      if (it == blocks.end()) {
        data.insert(data.end(), kBitListBlock, 0);
      } else {
        const auto& value = most_common(it->second);
        data.insert(data.end(), value.begin(), value.end());
      }
    }
    std::vector<uint8_t> b = unpack_bits(data);
    size_t set = 0;
    std::optional<size_t> first_set;
    size_t last_set = 0;
    for (size_t i = 0; i < b.size(); ++i) {
      if (b[i] == 0) continue;
      ++set;
      if (!first_set.has_value()) first_set = i;
      last_set = i;
    }
    const std::string name = fmt::format("bitliste_{}.bin", hex2(list));
    w.push_back(fmt::format(
        "  List {}: {} bits ({} blocks, {} missing), {} set, numbers {} -> {}",
        hex2(list), b.size(), height, height - blocks.size(), set,
        first_set.has_value() ? fmt::format("{}-{}", *first_set, last_set)
                              : std::string("-"),
        name));
    report.outputs.push_back(
        {"dienste/" + name, std::move(data), std::nullopt});
    bits[list] = std::move(b);
  }
  const auto list0 = bits.find(0x00);
  if (list0 != bits.end()) {
    const std::vector<uint8_t>& b0 = list0->second;
    size_t clear = 0;
    std::optional<size_t> first_clear;
    size_t last_clear = 0;
    for (size_t i = 0; i < b0.size(); ++i) {
      if (b0[i] != 0) continue;
      ++clear;
      if (!first_clear.has_value()) first_clear = i;
      last_clear = i;
    }
    if (clear > 0) {
      std::string others;
      for (const auto& [list, b] : bits) {
        if (list == 0x00) continue;
        size_t overlap = 0;
        for (size_t i = 0; i < std::min(b.size(), b0.size()); ++i) {
          overlap += (b[i] != 0 && b0[i] == 0) ? 1 : 0;
        }
        if (!others.empty()) others += ", ";
        others += fmt::format("{}: {}", hex2(list), overlap);
      }
      w.push_back(fmt::format(
          "    List 00: {} numbers NOT set ({}-{}); of these in other lists: "
          "{} -> list 00 acts as a block list (0 = blocked)",
          clear, *first_clear, last_clear, others));
    }
  }
  for (const auto& [a, ba] : bits) {
    for (const auto& [c, bc] : bits) {
      if (a == c) continue;
      const size_t n = std::min(ba.size(), bc.size());
      size_t in_a = 0;
      size_t outside = 0;
      for (size_t i = 0; i < n; ++i) {
        in_a += ba[i];
        outside += (ba[i] != 0 && bc[i] == 0) ? 1 : 0;
      }
      if (in_a > 0 && static_cast<double>(outside) <=
                          kSubsetTolerance * static_cast<double>(in_a)) {
        w.push_back(fmt::format(
            "    List {} is (almost) a subset of list {} ({} of {} bits "
            "outside)",
            hex2(a), hex2(c), outside, in_a));
      }
    }
  }

  // --- Encrypted and unknown services ------------------------------------
  w.push_back("");
  w.push_back("OTHER SERVICES (raw data in dienst_*.bin)");
  const std::map<std::pair<uint8_t, uint8_t>, std::string> kDescriptions = {
      {{0x01, 0x05},
       "unknown; every second byte 7 bits (15-bit values), no repeats"},
      {{0x03, 0x07}, "account balance; VideoDat DES, 8 rounds"},
      {{0x03, 0x08}, "account balance; VideoDat DES, 8 rounds"},
      {{0x03, 0x0A}, "change of credit limit; VideoDat DES, 8 rounds"},
  };
  std::map<std::pair<uint8_t, uint8_t>, std::vector<const VideoDatPacket*>>
      groups;
  for (const VideoDatPacket* x : ok) {
    const std::pair<uint8_t, uint8_t> key{x->stream(), x->payload[0]};
    if (kDescriptions.count(key) != 0) groups[key].push_back(x);
  }
  for (const auto& [key, xs] : groups) {
    std::vector<const std::vector<uint8_t>*> distinct;
    for (const VideoDatPacket* x : xs) {
      if (std::none_of(distinct.begin(), distinct.end(),
                       [&](const auto* d) { return *d == x->payload; })) {
        distinct.push_back(&x->payload);
      }
    }
    const std::string name =
        fmt::format("dienst_{}_typ{}.bin", hex2(key.first), hex2(key.second));
    VideoDatOutputFile output{"dienste/" + name, {}, std::nullopt};
    for (const auto* payload : distinct) {
      output.data.push_back(static_cast<uint8_t>(payload->size() & 0xFF));
      output.data.push_back(static_cast<uint8_t>(payload->size() >> 8));
      output.data.insert(output.data.end(), payload->begin(), payload->end());
    }
    report.outputs.push_back(std::move(output));
    w.push_back(fmt::format(
        "  Stream {} type {}: {} packets, {} distinct, {} - {}; {} -> {}",
        hex2(key.first), hex2(key.second), xs.size(), distinct.size(),
        time_or_unknown(clock.at(xs.front()->position)),
        time_or_unknown(clock.at(xs.back()->position)), kDescriptions.at(key),
        name));
  }
  w.push_back(
      "  (format of the .bin files: per packet 2 bytes length LE + "
      "payload)");

  VideoDatOutputFile text{"dienste/dienste.txt", {}, std::nullopt};
  for (const std::string& line : w) {
    text.data.insert(text.data.end(), line.begin(), line.end());
    text.data.push_back('\n');
  }
  report.outputs.push_back(std::move(text));
  return report;
}

}  // namespace orc
