/*
 * File:        videodat_packet.cpp
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     VideoDat packet framing, descrambling and multi-copy repair
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_packet.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <optional>
#include <set>

namespace orc {

namespace {

constexpr uint8_t kSync0 = 0x56;
constexpr uint8_t kSync1 = 0xAE;

// Sync, stream, control, counter, three permutation bytes and the key.
constexpr size_t kHeaderBytes = 9;
// Header plus the two checksum bytes.
constexpr size_t kOverheadBytes = kHeaderBytes + 2;

// The key byte is transmitted xor this.
constexpr uint8_t kKeyMask = 0x58;

// An escaped sync word inside a packet is followed by this byte, which is not
// part of the packet.
constexpr uint8_t kSyncEscape = 0xFF;

// Payload blocks are permuted in groups of this many bytes.
constexpr size_t kPermutationBlock = 8;

// Soft-decision repair tries at most this many disagreeing bits; 2^12
// combinations.
constexpr size_t kMaximumDisagreeingBits = 12;

// Byte-combination repair tries at most this many differing positions; 2^14
// combinations.
constexpr size_t kMaximumDifferingBytes = 14;

// Bytes one lost frame's worth of a packet occupies: six lines of ten.
constexpr size_t kFrameBytes = 60;

// First bit of the payload: header bits are common to every copy.
constexpr size_t kFirstPayloadBit = kHeaderBytes * 8;

std::array<uint32_t, kPermutationBlock> permutation(uint8_t d, uint8_t e,
                                                    uint8_t f) {
  const uint32_t e_bit = (e >> 2) & 1u;
  const uint32_t d_bits = d & 7u;
  const uint32_t f_bits = (f >> 1) & 3u;
  const uint32_t multiplier = (2u * f_bits + 1u + 6u * e_bit) % 8u;
  std::array<uint32_t, kPermutationBlock> q{};
  for (uint32_t c = 0; c < kPermutationBlock; ++c) {
    q[c] = (((c ^ e_bit) + (d_bits ^ e_bit) + 1u) * multiplier) % 8u;
  }
  return q;
}

uint32_t stored_checksum(const std::vector<uint8_t>& raw, size_t n) {
  return static_cast<uint32_t>(raw[kHeaderBytes + n]) |
         (static_cast<uint32_t>(raw[kHeaderBytes + n + 1]) << 8);
}

// Raw bytes from soft values: a bit is one where its soft value is not
// negative, bits least significant first, every byte inverted.
std::vector<uint8_t> raw_from_soft(const std::vector<int32_t>& soft) {
  std::vector<uint8_t> raw(soft.size() / 8);
  for (size_t byte = 0; byte < raw.size(); ++byte) {
    uint8_t value = 0;
    for (size_t bit = 0; bit < 8; ++bit) {
      if (soft[byte * 8 + bit] >= 0) {
        value = static_cast<uint8_t>(value | (1u << bit));
      }
    }
    raw[byte] = static_cast<uint8_t>(value ^ 0xFFu);
  }
  return raw;
}

// A packet read with the escape byte after every embedded sync word removed.
// The stream indexes the packet's bytes came from are kept, so that its soft
// values still address the right bits.
struct UnstuffedPacket {
  std::vector<uint8_t> raw;
  std::vector<size_t> indexes;
  size_t end = 0;
};

UnstuffedPacket unstuff_packet(const std::vector<uint8_t>& stream,
                               size_t start) {
  UnstuffedPacket packet;
  packet.raw = {stream[start], stream[start + 1]};
  packet.indexes = {start, start + 1};
  size_t position = start + 2;
  std::optional<size_t> length;
  while (position < stream.size() &&
         (!length.has_value() || packet.raw.size() < *length)) {
    packet.raw.push_back(stream[position]);
    packet.indexes.push_back(position);
    ++position;
    if (packet.raw.size() == 4) {
      length = videodat_packet_length(packet.raw[3]);
    }
    const size_t size = packet.raw.size();
    if (size > 3 && packet.raw[size - 2] == kSync0 &&
        packet.raw[size - 1] == kSync1 && position < stream.size() &&
        stream[position] == kSyncEscape) {
      ++position;
    }
  }
  packet.end = position;
  return packet;
}

// Soft decision over the copies of a packet (see repair_videodat_packets).
std::optional<std::vector<uint8_t>> soft_repair(
    const std::vector<VideoDatPacket*>& group) {
  std::vector<const VideoDatPacket*> copies;
  for (const VideoDatPacket* packet : group) {
    if (!packet->soft.empty() &&
        packet->soft.size() == 8 * packet->raw.size()) {
      copies.push_back(packet);
    }
  }
  if (copies.empty()) return std::nullopt;

  const size_t bits = copies.front()->soft.size();
  std::vector<int32_t> total(bits, 0);
  for (const VideoDatPacket* packet : copies) {
    for (size_t bit = 0; bit < bits; ++bit) {
      total[bit] += packet->soft[bit];
    }
  }
  std::vector<uint8_t> raw = raw_from_soft(total);
  // Sync and header are common to every copy.
  std::copy(copies.front()->raw.begin(),
            copies.front()->raw.begin() + kHeaderBytes, raw.begin());
  if (videodat_checksum_ok(raw)) return raw;
  if (copies.size() < 2) return std::nullopt;

  // Payload bits on which the copies disagree.
  std::vector<size_t> disagreeing;
  for (size_t bit = kFirstPayloadBit; bit < bits; ++bit) {
    bool any_one = false;
    bool any_zero = false;
    for (const VideoDatPacket* packet : copies) {
      (packet->soft[bit] >= 0 ? any_one : any_zero) = true;
    }
    if (any_one && any_zero) disagreeing.push_back(bit);
  }
  if (disagreeing.empty() || disagreeing.size() > kMaximumDisagreeingBits) {
    return std::nullopt;
  }

  // Every combination of flips; exactly one distinct passing result is
  // accepted.
  std::optional<std::vector<uint8_t>> solution;
  const uint32_t combinations = 1u << disagreeing.size();
  for (uint32_t mask = 1; mask < combinations; ++mask) {
    std::vector<uint8_t> candidate = raw;
    for (size_t index = 0; index < disagreeing.size(); ++index) {
      if (mask & (1u << index)) {
        const size_t bit = disagreeing[index];
        candidate[bit / 8] =
            static_cast<uint8_t>(candidate[bit / 8] ^ (1u << (bit % 8)));
      }
    }
    if (!videodat_checksum_ok(candidate)) continue;
    if (solution.has_value() && *solution != candidate) return std::nullopt;
    solution = std::move(candidate);
  }
  return solution;
}

// Combine whole bytes of copies that differ in few positions.
std::optional<std::vector<uint8_t>> byte_repair(
    const std::vector<VideoDatPacket*>& copies) {
  const size_t length = copies.front()->raw.size();
  std::vector<size_t> differing;
  std::vector<std::array<uint8_t, 2>> values;
  for (size_t position = 0; position < length; ++position) {
    std::set<uint8_t> seen;
    for (const VideoDatPacket* packet : copies) {
      seen.insert(packet->raw[position]);
    }
    if (seen.size() == 1) continue;
    if (seen.size() > 2) return std::nullopt;
    differing.push_back(position);
    values.push_back({*seen.begin(), *seen.rbegin()});
  }
  if (differing.empty() || differing.size() > kMaximumDifferingBytes) {
    return std::nullopt;
  }

  std::vector<uint8_t> base = copies.front()->raw;
  for (size_t index = 0; index < differing.size(); ++index) {
    base[differing[index]] = values[index][0];
  }
  std::optional<std::vector<uint8_t>> solution;
  size_t hits = 0;
  const uint32_t combinations = 1u << differing.size();
  for (uint32_t mask = 0; mask < combinations; ++mask) {
    std::vector<uint8_t> candidate = base;
    for (size_t index = 0; index < differing.size(); ++index) {
      candidate[differing[index]] = values[index][(mask >> index) & 1u];
    }
    if (videodat_checksum_ok(candidate)) {
      ++hits;
      solution = std::move(candidate);
    }
  }
  return (hits == 1) ? solution : std::nullopt;
}

// Head of one copy and tail of another, or one copy with a lost frame filled
// in from the other.
std::optional<std::vector<uint8_t>> pair_repair(const std::vector<uint8_t>& a,
                                                const std::vector<uint8_t>& b) {
  const size_t length = a.size();
  const size_t n = videodat_packet_length(a[3]) - kOverheadBytes;
  const uint8_t key = static_cast<uint8_t>(a[8] ^ kKeyMask);
  const int64_t base = static_cast<int64_t>(a[2]) + a[3] + a[4];
  const size_t checksum_lo = kHeaderBytes + n;
  const size_t checksum_hi = kHeaderBytes + n + 1;

  std::vector<int64_t> sum_a(n + 1, 0);
  std::vector<int64_t> sum_b(n + 1, 0);
  for (size_t i = 0; i < n; ++i) {
    sum_a[i + 1] = sum_a[i] + (a[kHeaderBytes + i] ^ key);
    sum_b[i + 1] = sum_b[i] + (b[kHeaderBytes + i] ^ key);
  }

  std::set<std::vector<uint8_t>> solutions;

  // 1) a[:s] + b[s:]
  for (size_t s = kHeaderBytes; s < length; ++s) {
    const size_t j = std::min(s - kHeaderBytes, n);
    const int64_t total = base + sum_a[j] + (sum_b[n] - sum_b[j]);
    const uint32_t checksum =
        static_cast<uint32_t>(checksum_lo < s ? a[checksum_lo]
                                              : b[checksum_lo]) |
        (static_cast<uint32_t>(checksum_hi < s ? a[checksum_hi]
                                               : b[checksum_hi])
         << 8);
    if ((static_cast<uint32_t>(total) & 0xFFFFu) != checksum) continue;
    std::vector<uint8_t> candidate(a.begin(),
                                   a.begin() + static_cast<ptrdiff_t>(s));
    candidate.insert(candidate.end(), b.begin() + static_cast<ptrdiff_t>(s),
                     b.end());
    if (videodat_checksum_ok(candidate)) solutions.insert(candidate);
  }

  // 2) a[:s] + b[s:s+60] + a[s:L-60]
  if (length > kFrameBytes + 10) {
    // Payload sums of a shifted back by one frame: shifted[i] = a[i - 60].
    std::vector<int64_t> sum_shifted(n + 1, 0);
    for (size_t i = 0; i < n; ++i) {
      const size_t position = kHeaderBytes + i;
      const uint8_t value =
          (position >= kFrameBytes) ? a[position - kFrameBytes] : 0;
      sum_shifted[i + 1] = sum_shifted[i] + (value ^ key);
    }
    for (size_t s = kHeaderBytes; s < length - kFrameBytes; ++s) {
      const size_t j1 = std::min(s - kHeaderBytes, n);
      const size_t j2 = std::min(s + kFrameBytes - kHeaderBytes, n);
      const int64_t total = base + sum_a[j1] + (sum_b[j2] - sum_b[j1]) +
                            (sum_shifted[n] - sum_shifted[j2]);
      const auto pick = [&](size_t p) -> uint8_t {
        if (p < s) return a[p];
        if (p < s + kFrameBytes) return b[p];
        return a[p - kFrameBytes];
      };
      const uint32_t checksum = static_cast<uint32_t>(pick(checksum_lo)) |
                                (static_cast<uint32_t>(pick(checksum_hi)) << 8);
      if ((static_cast<uint32_t>(total) & 0xFFFFu) != checksum) continue;
      std::vector<uint8_t> candidate(a.begin(),
                                     a.begin() + static_cast<ptrdiff_t>(s));
      candidate.insert(candidate.end(), b.begin() + static_cast<ptrdiff_t>(s),
                       b.begin() + static_cast<ptrdiff_t>(s + kFrameBytes));
      candidate.insert(
          candidate.end(), a.begin() + static_cast<ptrdiff_t>(s),
          a.begin() + static_cast<ptrdiff_t>(length - kFrameBytes));
      if (videodat_checksum_ok(candidate)) solutions.insert(candidate);
    }
  }

  if (solutions.size() != 1) return std::nullopt;
  return *solutions.begin();
}

}  // namespace

int8_t videodat_quantise_soft(double soft) {
  const double scaled = std::round(soft * kVideoDatSoftScale);
  if (soft >= 0.0) {
    return static_cast<int8_t>(std::clamp(scaled, 0.0, 127.0));
  }
  // A negative decision must stay negative, however close to the threshold.
  return static_cast<int8_t>(std::clamp(scaled, -127.0, -1.0));
}

void VideoDatByteStream::append(const VideoDatSlicedLine& line) {
  data.insert(data.end(), line.bytes.begin(), line.bytes.end());
  for (const float value : line.soft) {
    soft.push_back(videodat_quantise_soft(value));
  }
}

size_t videodat_packet_length(uint8_t control) {
  return (static_cast<size_t>(1) << (control & 0x0Fu)) + kOverheadBytes;
}

bool videodat_descramble(const std::vector<uint8_t>& raw,
                         std::vector<uint8_t>& out_payload) {
  out_payload.clear();
  if (raw.size() < kHeaderBytes) return false;
  const size_t n = static_cast<size_t>(1) << (raw[3] & 0x0Fu);
  if (raw.size() < n + kOverheadBytes) return false;

  const uint8_t key = static_cast<uint8_t>(raw[8] ^ kKeyMask);
  out_payload.assign(n, 0);
  if (n < kPermutationBlock) {
    // Very short packets are not permuted.
    for (size_t i = 0; i < n; ++i) {
      out_payload[i] = static_cast<uint8_t>(raw[kHeaderBytes + i] ^ key);
    }
  } else {
    // The 8-byte windows start one byte into the payload, so the last byte of
    // a window is column 0 of the next block.
    const auto q = permutation(raw[5], raw[6], raw[7]);
    for (size_t i = 0; i < n; ++i) {
      const size_t group = i / kPermutationBlock;
      const size_t column = i % kPermutationBlock;
      const size_t position =
          kHeaderBytes + kPermutationBlock * group +
          (q[column] + kPermutationBlock - 1) % kPermutationBlock;
      out_payload[i] = (position < kHeaderBytes + n)
                           ? static_cast<uint8_t>(raw[position] ^ key)
                           : 0;
    }
  }

  uint32_t sum = static_cast<uint32_t>(raw[2]) + raw[3] + raw[4];
  for (const uint8_t value : out_payload) sum += value;
  return (sum & 0xFFFFu) == stored_checksum(raw, n);
}

bool videodat_checksum_ok(const std::vector<uint8_t>& raw) {
  if (raw.size() < kHeaderBytes) return false;
  const size_t n = static_cast<size_t>(1) << (raw[3] & 0x0Fu);
  if (raw.size() < n + kOverheadBytes) return false;
  const uint8_t key = static_cast<uint8_t>(raw[8] ^ kKeyMask);
  uint32_t sum = static_cast<uint32_t>(raw[2]) + raw[3] + raw[4];
  for (size_t i = 0; i < n; ++i) {
    sum += static_cast<uint8_t>(raw[kHeaderBytes + i] ^ key);
  }
  return (sum & 0xFFFFu) == stored_checksum(raw, n);
}

std::vector<VideoDatPacket> find_videodat_packets(
    const VideoDatByteStream& stream) {
  std::vector<VideoDatPacket> packets;
  const std::vector<uint8_t>& data = stream.data;
  const bool have_soft = stream.soft.size() == data.size() * 8;
  size_t valid_end = 0;

  // Non-overlapping occurrences of the sync word, in order.
  size_t search = 0;
  while (search + 1 < data.size()) {
    if (!(data[search] == kSync0 && data[search + 1] == kSync1)) {
      ++search;
      continue;
    }
    const size_t position = search;
    search += 2;
    if (position < valid_end) continue;
    if (position + kHeaderBytes > data.size()) break;
    const size_t length = videodat_packet_length(data[position + 3]);
    if (position + length > data.size()) continue;

    VideoDatPacket packet;
    packet.position = position;
    packet.raw.assign(data.begin() + static_cast<ptrdiff_t>(position),
                      data.begin() + static_cast<ptrdiff_t>(position + length));
    packet.ok = videodat_descramble(packet.raw, packet.payload);
    if (have_soft) {
      packet.soft.assign(
          stream.soft.begin() + static_cast<ptrdiff_t>(8 * position),
          stream.soft.begin() +
              static_cast<ptrdiff_t>(8 * (position + length)));
    }

    if (packet.ok) {
      packet.recovery = VideoDatRecovery::kReceived;
      valid_end = position + length;
    } else {
      UnstuffedPacket unstuffed = unstuff_packet(data, position);
      if (unstuffed.end - position > unstuffed.raw.size()) {
        std::vector<uint8_t> payload;
        if (videodat_descramble(unstuffed.raw, payload)) {
          packet.raw = std::move(unstuffed.raw);
          packet.payload = std::move(payload);
          packet.ok = true;
          packet.recovery = VideoDatRecovery::kUnstuffed;
          if (have_soft) {
            packet.soft.clear();
            for (const size_t index : unstuffed.indexes) {
              for (size_t bit = 0; bit < 8; ++bit) {
                packet.soft.push_back(stream.soft[index * 8 + bit]);
              }
            }
          }
          valid_end = unstuffed.end;
        }
      }
    }
    packets.push_back(std::move(packet));
  }
  return packets;
}

std::vector<VideoDatPacket> find_videodat_packets(
    const std::vector<uint8_t>& data) {
  VideoDatByteStream stream;
  stream.data = data;
  return find_videodat_packets(stream);
}

VideoDatRepairCounts repair_videodat_packets(
    std::vector<VideoDatPacket>& packets) {
  // Copies share their stream, control, counter, permutation and key bytes;
  // groups are kept in order of first appearance.
  std::vector<std::vector<uint8_t>> keys;
  std::map<std::vector<uint8_t>, std::vector<VideoDatPacket*>> groups;
  for (VideoDatPacket& packet : packets) {
    if (packet.raw.size() < kHeaderBytes) continue;
    std::vector<uint8_t> key(packet.raw.begin() + 2,
                             packet.raw.begin() + kHeaderBytes);
    auto [it, inserted] = groups.try_emplace(key);
    if (inserted) keys.push_back(key);
    it->second.push_back(&packet);
  }

  VideoDatRepairCounts counts;
  for (const auto& key : keys) {
    std::vector<VideoDatPacket*>& group = groups[key];
    if (std::any_of(group.begin(), group.end(),
                    [](const VideoDatPacket* p) { return p->ok; })) {
      continue;
    }
    size_t length = 0;
    for (const VideoDatPacket* packet : group) {
      length = std::max(length, packet->raw.size());
    }
    std::vector<VideoDatPacket*> copies;
    for (VideoDatPacket* packet : group) {
      if (packet->raw.size() == length) copies.push_back(packet);
    }

    VideoDatRecovery recovery = VideoDatRecovery::kSoft;
    std::optional<std::vector<uint8_t>> good = soft_repair(copies);
    if (!good.has_value() && copies.size() < 2) continue;
    if (!good.has_value()) {
      recovery = VideoDatRecovery::kCopies;
      good = byte_repair(copies);
    }
    for (size_t i = 0; !good.has_value() && i < copies.size(); ++i) {
      for (size_t j = 0; !good.has_value() && j < copies.size(); ++j) {
        if (i == j) continue;
        good = pair_repair(copies[i]->raw, copies[j]->raw);
      }
    }
    if (!good.has_value()) continue;

    std::vector<uint8_t> payload;
    if (!videodat_descramble(*good, payload)) continue;
    for (VideoDatPacket* packet : copies) {
      packet->raw = *good;
      packet->payload = payload;
      packet->ok = true;
      packet->recovery = recovery;
    }
    if (recovery == VideoDatRecovery::kSoft) {
      ++counts.soft;
    } else {
      ++counts.copies;
    }
  }
  return counts;
}

}  // namespace orc
