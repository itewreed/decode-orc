/*
 * File:        videodat_file_assembler.cpp
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Reassembles the files carried in VideoDat packets
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_file_assembler.h"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <map>
#include <set>
#include <utility>

namespace orc {

namespace {

// Larger sizes in a file header are taken to be damage.
constexpr uint32_t kMaximumFileSize = 64u * 1024u * 1024u;

// Payload types.
constexpr uint8_t kTypeInfo = 0x00;
constexpr uint8_t kTypeHeader = 0x02;
constexpr uint8_t kTypeEnd = 0x03;
constexpr uint8_t kTypeData = 0x04;
constexpr uint8_t kTypeHeaderAlt = 0x0C;

// Block size when a stream carries no data packet to measure it from.
constexpr size_t kDefaultBlockSize = 255;

// Counter steps accepted as continuing the current section.
constexpr int64_t kCounterBack = 16;
constexpr int64_t kCounterForward = 128;
// Packets looked at after an unexpected counter to tell a cut from an
// outlier.
constexpr size_t kCounterLookahead = 3;
// Gap between the counters of two sections.
constexpr int64_t kSectionGap = 1024;

// A damaged header is placed against the next good packet at most this many
// bytes after it, and at most this many counter steps before it.
constexpr size_t kRescueMaximumDistance = 4000;
constexpr int64_t kRescueCounterReach = 8;
// Counter steps after a recovered header searched for the end of its file.
constexpr int64_t kRescueEndSlack = 64;
// File name bytes from this offset of a header on are voted with a preference
// for characters seen in file names.
constexpr size_t kHeaderNameOffset = 14;

// Counter gap beyond which data without a header starts a new run.
constexpr int64_t kOrphanGap = 512;

constexpr std::array<uint16_t, 128> kCp437High = {
    0x00C7, 0x00FC, 0x00E9, 0x00E2, 0x00E4, 0x00E0, 0x00E5, 0x00E7, 0x00EA,
    0x00EB, 0x00E8, 0x00EF, 0x00EE, 0x00EC, 0x00C4, 0x00C5, 0x00C9, 0x00E6,
    0x00C6, 0x00F4, 0x00F6, 0x00F2, 0x00FB, 0x00F9, 0x00FF, 0x00D6, 0x00DC,
    0x00A2, 0x00A3, 0x00A5, 0x20A7, 0x0192, 0x00E1, 0x00ED, 0x00F3, 0x00FA,
    0x00F1, 0x00D1, 0x00AA, 0x00BA, 0x00BF, 0x2310, 0x00AC, 0x00BD, 0x00BC,
    0x00A1, 0x00AB, 0x00BB, 0x2591, 0x2592, 0x2593, 0x2502, 0x2524, 0x2561,
    0x2562, 0x2556, 0x2555, 0x2563, 0x2551, 0x2557, 0x255D, 0x255C, 0x255B,
    0x2510, 0x2514, 0x2534, 0x252C, 0x251C, 0x2500, 0x253C, 0x255E, 0x255F,
    0x255A, 0x2554, 0x2569, 0x2566, 0x2560, 0x2550, 0x256C, 0x2567, 0x2568,
    0x2564, 0x2565, 0x2559, 0x2558, 0x2552, 0x2553, 0x256B, 0x256A, 0x2518,
    0x250C, 0x2588, 0x2584, 0x258C, 0x2590, 0x2580, 0x03B1, 0x00DF, 0x0393,
    0x03C0, 0x03A3, 0x03C3, 0x00B5, 0x03C4, 0x03A6, 0x0398, 0x03A9, 0x03B4,
    0x221E, 0x03C6, 0x03B5, 0x2229, 0x2261, 0x00B1, 0x2265, 0x2264, 0x2320,
    0x2321, 0x00F7, 0x2248, 0x00B0, 0x2219, 0x00B7, 0x221A, 0x207F, 0x00B2,
    0x25A0, 0x00A0,
};

using Bytes = std::vector<uint8_t>;

// Counts of distinct byte strings, remembering the order they were first
// seen in: of equally frequent values the first seen wins.
class VoteCounter {
 public:
  void add(const Bytes& value, size_t count) {
    for (auto& entry : entries_) {
      if (entry.first == value) {
        entry.second += count;
        return;
      }
    }
    entries_.emplace_back(value, count);
  }
  void merge(const VoteCounter& other) {
    for (const auto& entry : other.entries_) add(entry.first, entry.second);
  }
  bool empty() const { return entries_.empty(); }
  size_t distinct() const { return entries_.size(); }
  const Bytes& most_common() const {
    const auto* best = &entries_.front();
    for (const auto& entry : entries_) {
      if (entry.second > best->second) best = &entry;
    }
    return best->first;
  }
  const std::vector<std::pair<Bytes, size_t>>& entries() const {
    return entries_;
  }

 private:
  std::vector<std::pair<Bytes, size_t>> entries_;
};

uint32_t read_le32(const Bytes& bytes, size_t offset) {
  return static_cast<uint32_t>(bytes[offset]) |
         static_cast<uint32_t>(bytes[offset + 1]) << 8 |
         static_cast<uint32_t>(bytes[offset + 2]) << 16 |
         static_cast<uint32_t>(bytes[offset + 3]) << 24;
}

void append_utf8(std::string& out, uint32_t code_point) {
  if (code_point < 0x80) {
    out += static_cast<char>(code_point);
  } else if (code_point < 0x800) {
    out += static_cast<char>(0xC0 | (code_point >> 6));
    out += static_cast<char>(0x80 | (code_point & 0x3F));
  } else {
    out += static_cast<char>(0xE0 | (code_point >> 12));
    out += static_cast<char>(0x80 | ((code_point >> 6) & 0x3F));
    out += static_cast<char>(0x80 | (code_point & 0x3F));
  }
}

bool is_name_char(uint8_t c) {
  static const std::string kExtra = "_!$#&@%'(){}~^`-";
  return std::isalnum(c) != 0 ||
         kExtra.find(static_cast<char>(c)) != std::string::npos;
}

// A DOS 8.3 name, as file headers carry them.
bool is_dos_name(const Bytes& name) {
  size_t i = 0;
  while (i < name.size() && is_name_char(name[i])) ++i;
  if (i < 1 || i > 8) return false;
  if (i == name.size()) return true;
  if (name[i] != '.') return false;
  const size_t extension = name.size() - i - 1;
  if (extension > 3) return false;
  for (size_t k = i + 1; k < name.size(); ++k) {
    if (!is_name_char(name[k])) return false;
  }
  return true;
}

// Characters a tie between header copies is settled in favour of.
bool is_preferred_name_char(uint8_t c) {
  static const std::string kExtra = "._$!#&-";
  return std::isalnum(c) != 0 ||
         kExtra.find(static_cast<char>(c)) != std::string::npos;
}

struct HeaderFields {
  Bytes name;
  uint32_t size = 0;
  uint32_t timestamp = 0;
};

// Name, size and date of a file header, or nullopt when implausible.
std::optional<HeaderFields> header_fields(const Bytes& payload) {
  if (payload.size() < 15 ||
      (payload[0] != kTypeHeader && payload[0] != kTypeHeaderAlt)) {
    return std::nullopt;
  }
  HeaderFields fields;
  fields.size = read_le32(payload, 5);
  fields.timestamp = read_le32(payload, 9);
  const size_t length = payload[13];
  if (length < 1 || length > 12 || 14 + length > payload.size() ||
      fields.size > kMaximumFileSize) {
    return std::nullopt;
  }
  fields.name.assign(payload.begin() + 14,
                     payload.begin() + 14 + static_cast<ptrdiff_t>(length));
  return fields;
}

// Bytewise majority over damaged copies of a header.
Bytes vote_header(const std::vector<Bytes>& copies) {
  size_t length = copies.front().size();
  for (const Bytes& copy : copies) length = std::min(length, copy.size());
  Bytes out(length, 0);
  for (size_t i = 0; i < length; ++i) {
    std::vector<std::pair<uint8_t, size_t>> counts;
    for (const Bytes& copy : copies) {
      auto it = std::find_if(counts.begin(), counts.end(),
                             [&](const auto& c) { return c.first == copy[i]; });
      if (it == counts.end()) {
        counts.emplace_back(copy[i], 1);
      } else {
        ++it->second;
      }
    }
    size_t best = 0;
    for (const auto& c : counts) best = std::max(best, c.second);
    std::vector<uint8_t> candidates;
    for (const auto& c : counts) {
      if (c.second == best) candidates.push_back(c.first);
    }
    if (candidates.size() > 1 && i >= kHeaderNameOffset) {
      std::vector<uint8_t> preferred;
      for (const uint8_t c : candidates) {
        if (is_preferred_name_char(c)) preferred.push_back(c);
      }
      if (!preferred.empty()) candidates = preferred;
    }
    out[i] = candidates.front();
  }
  return out;
}

size_t blocks_for(uint32_t size, size_t block_size) {
  return block_size == 0 ? 0 : (size + block_size - 1) / block_size;
}

std::string hex2(unsigned value) { return fmt::format("{:02X}", value); }

// One reception of a file, between its header and its end.
struct Transmission {
  std::string name;
  uint32_t size = 0;
  std::optional<VideoDatDateTime> modified;
  int64_t start = 0;
  size_t block_size = 0;
  size_t blocks = 0;
  std::map<size_t, VoteCounter> data;
};

// Every reception of one file, identified by name, size and date.
struct FileEntry {
  std::string name;
  uint32_t size = 0;
  std::optional<VideoDatDateTime> modified;
  uint8_t stream = 0;
  size_t block_size = 0;
  size_t blocks = 0;
  size_t count = 0;
  std::map<size_t, VoteCounter> data;
};

struct OrphanRun {
  std::map<int64_t, Bytes> blocks;
  std::optional<int64_t> end;
};

// A good packet of one sub-channel with its unwrapped counter.
struct Sequenced {
  const VideoDatPacket* packet;
  int64_t counter;
};

std::map<int64_t, Bytes> rescue_headers(
    const std::vector<const VideoDatPacket*>& bad,
    const std::vector<Sequenced>& good, const std::map<int64_t, Bytes>& seq,
    size_t block_size, uint8_t stream, std::vector<std::string>& log) {
  std::vector<int64_t> order;
  std::map<int64_t, std::vector<Bytes>> candidates;
  for (const VideoDatPacket* packet : bad) {
    if (!header_fields(packet->payload).has_value()) continue;
    const auto next =
        std::upper_bound(good.begin(), good.end(), packet->position,
                         [](size_t position, const Sequenced& s) {
                           return position < s.packet->position;
                         });
    if (next == good.end() ||
        next->packet->position - packet->position > kRescueMaximumDistance) {
      continue;
    }
    const size_t j = static_cast<size_t>(next - good.begin());
    const int64_t following = good[j].counter;
    const int64_t lowest =
        j > 0 ? good[j - 1].counter + 1 : following - kRescueCounterReach;
    std::optional<int64_t> counter;
    for (int64_t w = following - 1;
         w >= std::max(lowest, following - kRescueCounterReach); --w) {
      if ((static_cast<uint64_t>(w) & 0xFFu) == packet->counter()) {
        counter = w;
        break;
      }
    }
    if (!counter.has_value()) {
      counter = following - 1;
      const unsigned differing =
          (static_cast<uint64_t>(*counter) ^ packet->counter()) & 0xFFu;
      if (__builtin_popcount(differing) > 1) continue;
    }
    const auto existing = seq.find(*counter);
    if (existing != seq.end() && (existing->second[0] == kTypeHeader ||
                                  existing->second[0] == kTypeHeaderAlt)) {
      continue;  // a good header is already there
    }
    bool only_data_between = true;
    for (int64_t w = *counter + 1; w < following; ++w) {
      const auto it = seq.find(w);
      if (it != seq.end() && it->second[0] != kTypeData) {
        only_data_between = false;
        break;
      }
    }
    if (!only_data_between) continue;
    auto [it, inserted] = candidates.try_emplace(*counter);
    if (inserted) order.push_back(*counter);
    it->second.push_back(packet->payload);
  }

  std::map<int64_t, Bytes> rescued;
  for (const int64_t counter : order) {
    const std::vector<Bytes>& copies = candidates[counter];
    const Bytes payload =
        copies.size() > 1 ? vote_header(copies) : copies.front();
    const auto fields = header_fields(payload);
    if (!fields.has_value() || !is_dos_name(fields->name)) continue;
    if (!videodat_dos_time(fields->timestamp).has_value()) continue;
    // The number of data blocks up to an end mark must match the size.
    const size_t blocks = blocks_for(fields->size, block_size);
    std::optional<int64_t> end;
    for (int64_t v = counter + 1;
         v < counter + 1 + static_cast<int64_t>(blocks) + kRescueEndSlack;
         ++v) {
      const auto it = seq.find(v);
      if (it != seq.end() &&
          (it->second[0] == kTypeHeader || it->second[0] == kTypeEnd ||
           it->second[0] == kTypeHeaderAlt)) {
        end = v;
        break;
      }
    }
    if (end.has_value() && seq.at(*end)[0] == kTypeEnd && block_size != 0 &&
        *end - counter - 1 != static_cast<int64_t>(blocks)) {
      continue;
    }
    rescued[counter] = payload;
    log.push_back(fmt::format(
        "Stream {}: repaired damaged file header: {} ({} bytes, {} cop{})",
        hex2(stream),
        videodat_safe_name(fields->name.data(), fields->name.size()),
        fields->size, copies.size(), copies.size() > 1 ? "ies" : "y"));
  }
  return rescued;
}

std::string with_counter_suffix(const std::string& name, size_t number) {
  const size_t dot = name.rfind('.');
  if (dot == std::string::npos || dot == 0) {
    return fmt::format("{}_{}", name, number);
  }
  return fmt::format("{}_{}{}", name.substr(0, dot), number, name.substr(dot));
}

std::string ascii_lower(std::string text) {
  for (char& c : text) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return text;
}

bool is_header(const Bytes& payload) {
  return payload[0] == kTypeHeader || payload[0] == kTypeHeaderAlt;
}

}  // namespace

bool operator==(const VideoDatDateTime& a, const VideoDatDateTime& b) {
  return a.year == b.year && a.month == b.month && a.day == b.day &&
         a.hour == b.hour && a.minute == b.minute && a.second == b.second;
}

bool operator!=(const VideoDatDateTime& a, const VideoDatDateTime& b) {
  return !(a == b);
}

int64_t VideoDatDateTime::seconds_since_epoch() const {
  // Days from civil (proleptic Gregorian).
  const int64_t y = year - (month <= 2 ? 1 : 0);
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const int64_t yoe = y - era * 400;
  const int64_t mp = (month + 9) % 12;
  const int64_t doy = (153 * mp + 2) / 5 + day - 1;
  const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int64_t days = era * 146097 + doe - 719468;
  return days * 86400 + static_cast<int64_t>(hour) * 3600 +
         static_cast<int64_t>(minute) * 60 + second;
}

VideoDatDateTime VideoDatDateTime::from_seconds_since_epoch(int64_t seconds) {
  int64_t days = seconds / 86400;
  int64_t rest = seconds % 86400;
  if (rest < 0) {
    rest += 86400;
    --days;
  }
  // Civil from days.
  const int64_t z = days + 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const int64_t doe = z - era * 146097;
  const int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const int64_t mp = (5 * doy + 2) / 153;
  VideoDatDateTime out;
  out.day = static_cast<int>(doy - (153 * mp + 2) / 5 + 1);
  out.month = static_cast<int>(mp < 10 ? mp + 3 : mp - 9);
  out.year = static_cast<int>(yoe + era * 400 + (out.month <= 2 ? 1 : 0));
  out.hour = static_cast<int>(rest / 3600);
  out.minute = static_cast<int>((rest / 60) % 60);
  out.second = static_cast<int>(rest % 60);
  return out;
}

std::optional<VideoDatDateTime> make_videodat_date_time(int year, int month,
                                                        int day, int hour,
                                                        int minute,
                                                        int second) {
  static constexpr std::array<int, 12> kDays = {31, 28, 31, 30, 31, 30,
                                                31, 31, 30, 31, 30, 31};
  if (year < 1 || year > 9999 || month < 1 || month > 12 || day < 1 ||
      hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 ||
      second > 59) {
    return std::nullopt;
  }
  const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
  const int days_in_month = kDays[month - 1] + ((month == 2 && leap) ? 1 : 0);
  if (day > days_in_month) return std::nullopt;
  return VideoDatDateTime{year, month, day, hour, minute, second};
}

std::optional<VideoDatDateTime> videodat_dos_time(uint32_t value) {
  const uint32_t date = value >> 16;
  const uint32_t time = value & 0xFFFFu;
  return make_videodat_date_time(
      1980 + static_cast<int>(date >> 9), static_cast<int>((date >> 5) & 15),
      static_cast<int>(date & 31), static_cast<int>(time >> 11),
      static_cast<int>((time >> 5) & 63), static_cast<int>((time & 31) * 2));
}

std::string videodat_format_date(const VideoDatDateTime& value) {
  return fmt::format("{:02d}.{:02d}.{:04d}", value.day, value.month,
                     value.year);
}

std::string videodat_format_time(const VideoDatDateTime& value) {
  return fmt::format("{:02d}:{:02d}:{:02d}", value.hour, value.minute,
                     value.second);
}

std::string videodat_cp437_to_utf8(const uint8_t* bytes, size_t count) {
  std::string out;
  for (size_t i = 0; i < count && bytes[i] != 0; ++i) {
    append_utf8(out, bytes[i] < 0x80 ? bytes[i] : kCp437High[bytes[i] - 0x80]);
  }
  return out;
}

std::string videodat_safe_name(const uint8_t* bytes, size_t count) {
  static const std::string kReserved = "\\/:*?\"<>|";
  std::string out;
  for (size_t i = 0; i < count && bytes[i] != 0; ++i) {
    const uint8_t c = bytes[i];
    if (c < 0x20 || kReserved.find(static_cast<char>(c)) != std::string::npos) {
      out += '_';
    } else {
      append_utf8(out, c < 0x80 ? c : kCp437High[c - 0x80]);
    }
  }
  const size_t first = out.find_first_not_of(" .");
  if (first == std::string::npos) return "unbenannt";
  const size_t last = out.find_last_not_of(" .");
  return out.substr(first, last - first + 1);
}

std::vector<std::optional<int64_t>> unwrap_videodat_counter(
    const std::vector<uint8_t>& counters) {
  const auto jump = [](uint8_t from, uint8_t to) {
    const int64_t d = (static_cast<int64_t>(to) - from) & 0xFF;
    return d > 128 ? d - 256 : d;
  };
  const auto continues = [](int64_t d) {
    return -kCounterBack <= d && d <= kCounterForward;
  };

  std::vector<std::optional<int64_t>> out;
  out.reserve(counters.size());
  bool have_last = false;
  uint8_t last_counter = 0;
  int64_t last_unwrapped = 0;
  int64_t highest = 0;
  for (size_t i = 0; i < counters.size(); ++i) {
    const uint8_t c = counters[i];
    int64_t u = c;
    if (have_last) {
      const int64_t d = jump(last_counter, c);
      if (continues(d)) {
        u = last_unwrapped + d;
      } else {
        // A cut, or an outlier?  Look at the packets that follow.
        size_t following = 0;
        size_t agreeing = 0;
        for (size_t k = i + 1;
             k < counters.size() && k <= i + kCounterLookahead; ++k) {
          ++following;
          if (continues(jump(c, counters[k]))) ++agreeing;
        }
        if (following > 0 && agreeing >= std::max<size_t>(1, following - 1)) {
          int64_t offset = (static_cast<int64_t>(c) - highest) % 256;
          if (offset < 0) offset += 256;
          u = highest + kSectionGap + offset;  // a new section
        } else {
          out.push_back(std::nullopt);
          continue;
        }
      }
    }
    highest = have_last ? std::max(highest, u) : u;
    have_last = true;
    last_counter = c;
    last_unwrapped = u;
    out.push_back(u);
  }
  return out;
}

VideoDatAssembly assemble_videodat_files(
    const std::vector<VideoDatPacket>& packets,
    const VideoDatAssemblyOptions& options) {
  VideoDatAssembly result;
  std::vector<std::string>& log = result.log;

  using SubChannel = std::pair<uint8_t, uint8_t>;
  std::map<SubChannel, std::vector<const VideoDatPacket*>> subs;
  std::map<SubChannel, std::vector<const VideoDatPacket*>> bad_subs;
  for (const VideoDatPacket& packet : packets) {
    const SubChannel key{packet.stream(), packet.sub_channel()};
    if (packet.ok) {
      subs[key].push_back(&packet);
    } else if (!packet.payload.empty() && is_header(packet.payload)) {
      bad_subs[key].push_back(&packet);
    }
  }

  std::vector<FileEntry> files;
  std::map<std::string, size_t> other_dumps;  // path -> index into outputs

  for (const auto& [key, received] : subs) {
    const uint8_t stream = key.first;
    const unsigned sub = key.second;

    std::vector<uint8_t> counters;
    counters.reserve(received.size());
    for (const VideoDatPacket* packet : received) {
      counters.push_back(packet->counter());
    }
    const auto unwrapped = unwrap_videodat_counter(counters);

    std::vector<Sequenced> good;
    std::map<int64_t, Bytes> seq;
    std::map<int64_t, VoteCounter> votes;
    for (size_t i = 0; i < received.size(); ++i) {
      if (!unwrapped[i].has_value()) continue;
      const int64_t u = *unwrapped[i];
      good.push_back({received[i], u});
      seq.emplace(u, received[i]->payload);
      // Unrepaired packets count double.
      votes[u].add(received[i]->payload, received[i]->unrepaired() ? 2 : 1);
    }

    // Block size: the largest data block of the stream (normally 255).  The
    // last block of a file is often sent in a smaller packet and must not
    // set it.
    size_t block_size = 0;
    bool any_data = false;
    for (const auto& entry : seq) {
      if (entry.second[0] == kTypeData) {
        block_size = std::max(block_size, entry.second.size() - 1);
        any_data = true;
      }
    }
    if (!any_data) block_size = kDefaultBlockSize;

    const auto bad_it = bad_subs.find(key);
    const std::map<int64_t, Bytes> rescued = rescue_headers(
        bad_it == bad_subs.end() ? std::vector<const VideoDatPacket*>{}
                                 : bad_it->second,
        good, seq, block_size, stream, log);
    result.rescued_headers += rescued.size();
    for (const auto& entry : rescued) seq[entry.first] = entry.second;

    std::map<uint8_t, size_t> other;
    std::vector<OrphanRun> orphan_runs;
    std::optional<size_t> orphan;  // index into orphan_runs
    std::optional<Transmission> current;

    // Close a reception and combine it with earlier receptions of the same
    // file (name, size and date).
    const auto finish = [&](std::optional<Transmission>& transmission) {
      if (!transmission.has_value()) return;
      Transmission& t = *transmission;
      auto it =
          std::find_if(files.begin(), files.end(), [&](const FileEntry& f) {
            return f.name == t.name && f.size == t.size &&
                   f.modified.has_value() == t.modified.has_value() &&
                   (!f.modified.has_value() || *f.modified == *t.modified);
          });
      if (it == files.end()) {
        FileEntry entry;
        entry.name = t.name;
        entry.size = t.size;
        entry.modified = t.modified;
        entry.stream = stream;
        entry.block_size = t.block_size;
        entry.blocks = t.blocks;
        files.push_back(std::move(entry));
        it = files.end() - 1;
      }
      if (it->data.empty() && !t.data.empty()) {
        it->block_size = t.block_size;
        it->blocks = t.blocks;
      }
      if (!t.data.empty() && it->block_size != t.block_size) {
        transmission.reset();
        return;  // a different block size: ignored
      }
      ++it->count;
      for (const auto& [index, counter] : t.data) {
        it->data[index].merge(counter);
      }
      transmission.reset();
    };

    for (const auto& [u, payload] : seq) {
      const uint8_t type = payload[0];
      if (is_header(payload) && payload.size() >= 14) {
        finish(current);
        orphan.reset();
        const uint32_t size = read_le32(payload, 5);
        if (size > kMaximumFileSize) {
          log.push_back(fmt::format(
              "Stream {}: ignored file header with implausible size {}",
              hex2(stream), size));
          continue;
        }
        const size_t name_length =
            std::min<size_t>(payload[13], payload.size() - 14);
        Transmission t;
        t.name = videodat_safe_name(payload.data() + 14, name_length);
        t.size = size;
        t.modified = videodat_dos_time(read_le32(payload, 9));
        t.start = u;
        t.block_size = block_size;
        t.blocks = blocks_for(size, block_size);
        current = std::move(t);
      } else if (type == kTypeData) {
        bool placed = false;
        if (current.has_value()) {
          const int64_t index = u - current->start - 1;
          if (index < 0 || index >= static_cast<int64_t>(current->blocks)) {
            // Data beyond the end of the file: the next file's header is
            // missing, so this is data without a header.
            finish(current);
          } else {
            VoteCounter& counter = current->data[static_cast<size_t>(index)];
            counter = VoteCounter();
            for (const auto& [value, count] : votes[u].entries()) {
              if (value[0] == kTypeData) {
                counter.add(Bytes(value.begin() + 1, value.end()), count);
              }
            }
            placed = true;
            if (index == static_cast<int64_t>(current->blocks) - 1) {
              finish(current);
            }
          }
        }
        if (!placed) {
          if (orphan.has_value() && !orphan_runs[*orphan].blocks.empty() &&
              u - orphan_runs[*orphan].blocks.rbegin()->first > kOrphanGap) {
            orphan.reset();  // a cut in the tape
          }
          if (!orphan.has_value()) {
            orphan_runs.emplace_back();
            orphan = orphan_runs.size() - 1;
          }
          orphan_runs[*orphan].blocks[u] =
              Bytes(payload.begin() + 1, payload.end());
        }
      } else if (type == kTypeEnd) {
        // End of a file, or announcement of the next.
        finish(current);
        if (orphan.has_value()) orphan_runs[*orphan].end = u;
        orphan.reset();
      } else {
        ++other[type];
        if (options.dump_other) {
          const std::string path = fmt::format(
              "andere_dienste/strom_{}_typ_{}.bin", hex2(stream), hex2(type));
          auto [it, inserted] =
              other_dumps.try_emplace(path, result.outputs.size());
          if (inserted) result.outputs.push_back({path, {}, std::nullopt});
          Bytes& data = result.outputs[it->second].data;
          data.insert(data.end(), payload.begin(), payload.end());
        }
      }
      if (type == kTypeInfo && payload.size() > 9 && payload[7] > 2 &&
          payload[7] < 40) {
        const size_t end = std::min<size_t>(8 + payload[7], payload.size());
        bool printable = true;
        for (size_t i = 8; i < end; ++i) {
          printable = printable && payload[i] >= 32 && payload[i] < 127;
        }
        if (printable) {
          log.push_back(fmt::format(
              "Stream {}: info block '{}'", hex2(stream),
              std::string(payload.begin() + 8,
                          payload.begin() + static_cast<ptrdiff_t>(end))));
        }
      }
    }
    finish(current);

    if (options.keep_partial) {
      for (const OrphanRun& run : orphan_runs) {
        if (run.blocks.empty()) continue;
        // Data without a header: sent before the recording started, or not
        // recoverable.
        const int64_t first = run.blocks.begin()->first;
        const int64_t last = run.blocks.rbegin()->first;
        size_t longest = 0;
        for (const auto& block : run.blocks) {
          longest = std::max(longest, block.second.size());
        }
        VideoDatOutputFile output;
        output.path =
            fmt::format("strom_{}/ohne_dateikopf_{}.bin.unvollstaendig",
                        hex2(stream), first);
        for (int64_t u = first; u <= last; ++u) {
          const auto it = run.blocks.find(u);
          if (it == run.blocks.end()) {
            output.data.insert(output.data.end(), longest, 0);
          } else {
            output.data.insert(output.data.end(), it->second.begin(),
                               it->second.end());
          }
        }
        const size_t slash = output.path.rfind('/');
        log.push_back(fmt::format(
            "Stream {}/{}: {} data blocks without a file header{} -> {}",
            hex2(stream), sub, run.blocks.size(),
            (run.end.has_value() && *run.end == last + 1)
                ? ", end of file received"
                : "",
            output.path.substr(slash + 1)));
        result.outputs.push_back(std::move(output));
      }
    }
    if (!other.empty()) {
      std::string list;
      for (const auto& [type, count] : other) {
        if (!list.empty()) list += ", ";
        list += fmt::format("{}:{}", hex2(type), count);
      }
      log.push_back(fmt::format("Stream {}/{}: other packet types {}",
                                hex2(stream), sub, list));
    }
  }

  // Write the files; different files of the same name get a number.
  std::map<std::pair<uint8_t, std::string>, size_t> used;
  for (const FileEntry& f : files) {
    Bytes data;
    size_t missing = 0;
    for (size_t i = 0; i < f.blocks; ++i) {
      const auto it = f.data.find(i);
      Bytes block;
      if (it != f.data.end() && !it->second.empty()) {
        block = it->second.most_common();
        if (it->second.distinct() > 1) ++result.conflicting_blocks;
      } else {
        ++missing;
      }
      block.resize(f.block_size, 0);
      data.insert(data.end(), block.begin(), block.end());
    }
    data.resize(std::min<size_t>(data.size(), f.size));
    const bool complete = missing == 0 && data.size() == f.size;

    const size_t number = ++used[{f.stream, ascii_lower(f.name)}];
    std::string file_name =
        number > 1 ? with_counter_suffix(f.name, number) : f.name;

    VideoDatFileResult file;
    file.stream = f.stream;
    file.name = f.name;
    file.size = f.size;
    file.missing_blocks = missing;
    file.blocks = f.blocks;
    file.transmissions = f.count;
    if (complete || options.keep_partial) {
      if (!complete) file_name += ".unvollstaendig";
      file.output = result.outputs.size();
      result.outputs.push_back(
          {fmt::format("strom_{}/{}", hex2(f.stream), file_name),
           std::move(data), f.modified});
    }
    result.files.push_back(std::move(file));
  }
  if (result.rescued_headers > 0) {
    log.push_back(fmt::format("{} damaged file headers repaired",
                              result.rescued_headers));
  }
  if (result.conflicting_blocks > 0) {
    log.push_back(fmt::format(
        "{} file blocks were received in differing versions (majority vote)",
        result.conflicting_blocks));
  }
  return result;
}

std::optional<VideoDatChvdHeader> videodat_chvd_header(
    const std::vector<uint8_t>& data) {
  static const std::string kMagic = "CHVDFILE";
  if (data.size() < 32 ||
      !std::equal(kMagic.begin(), kMagic.end(), data.begin())) {
    return std::nullopt;
  }
  const uint32_t time = static_cast<uint32_t>(data[11] | data[12] << 8);
  const uint32_t date = static_cast<uint32_t>(data[13] | data[14] << 8);
  VideoDatChvdHeader header;
  header.length = read_le32(data, 15);
  header.modified = videodat_dos_time(date << 16 | time);
  for (size_t i = 20; i < 32 && data[i] != 0; ++i) {
    header.name += (data[i] < 0x80) ? static_cast<char>(data[i]) : '?';
  }
  return header;
}

}  // namespace orc
