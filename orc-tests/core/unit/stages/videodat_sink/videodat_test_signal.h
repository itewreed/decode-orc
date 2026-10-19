/*
 * File:        videodat_test_signal.h
 * Module:      orc-core-tests
 * Purpose:     Synthetic VideoDat packets, lines and recordings for unit tests
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_TESTS_VIDEODAT_TEST_SIGNAL_H
#define ORC_TESTS_VIDEODAT_TEST_SIGNAL_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "videodat_line_slicer.h"
#include "videodat_packet.h"
#include "videodat_stream_decoder.h"

namespace orc_unit_test {

// Key byte used by the test packets: k = 0x21.
inline constexpr uint8_t kTestKeyByte = 0x58 ^ 0x21;

// A packet with the given clear payload (a power of two bytes, at least 8).
// D = 7, E = 0, F = 0 make the block permutation the identity, which leaves
// only the one-byte rotation of the 8-byte windows to undo.
inline std::vector<uint8_t> make_videodat_packet(
    uint8_t stream, uint8_t control, uint8_t counter,
    const std::vector<uint8_t>& payload) {
  const uint8_t key = kTestKeyByte ^ 0x58;
  std::vector<uint8_t> raw = {0x56, 0xAE, stream, control,     counter,
                              0x07, 0x00, 0x00,   kTestKeyByte};
  const size_t n = payload.size();
  std::vector<uint8_t> scrambled(n, 0);
  if (n < 8) {
    for (size_t i = 0; i < n; ++i) {
      scrambled[i] = static_cast<uint8_t>(payload[i] ^ key);
    }
  } else {
    for (size_t g = 0; g < n / 8; ++g) {
      for (size_t j = 0; j < 8; ++j) {
        scrambled[8 * g + j] =
            static_cast<uint8_t>(payload[8 * g + (j + 1) % 8] ^ key);
      }
    }
  }
  raw.insert(raw.end(), scrambled.begin(), scrambled.end());
  uint32_t sum = static_cast<uint32_t>(stream) + control + counter;
  for (const uint8_t value : payload) sum += value;
  raw.push_back(static_cast<uint8_t>(sum & 0xFF));
  raw.push_back(static_cast<uint8_t>((sum >> 8) & 0xFF));
  return raw;
}

// Control byte for a payload of 2^n bytes on a sub-channel.
inline uint8_t videodat_control(unsigned n, unsigned sub_channel = 0) {
  return static_cast<uint8_t>((sub_channel << 7) | n);
}

// True when the bytes hold the packet sync word anywhere.
inline bool holds_sync(const std::vector<uint8_t>& bytes) {
  for (size_t i = 0; i + 1 < bytes.size(); ++i) {
    if (bytes[i] == 0x56 && bytes[i + 1] == 0xAE) return true;
  }
  return false;
}

// A received, checked packet as the assembler sees it.
inline orc::VideoDatPacket received_packet(uint8_t stream, uint8_t control,
                                           uint8_t counter,
                                           std::vector<uint8_t> payload,
                                           size_t position) {
  orc::VideoDatPacket packet;
  packet.position = position;
  packet.raw = make_videodat_packet(stream, control, counter, payload);
  packet.payload = std::move(payload);
  packet.ok = true;
  packet.recovery = orc::VideoDatRecovery::kReceived;
  return packet;
}

// Levels and timing of synthetic lines, in 10-bit CVBS counts and 4 x fsc
// samples.
struct SyntheticLineConfig {
  double low = 256.0;
  double high = 690.0;
  double start_edge = 223.4;
  bool with_sync = true;
  double sync_tip = 4.0;
  size_t sync_end = 83;
  size_t length = 1135;
};

// A VBI line carrying ten bytes: start bit high, data bits inverted least
// significant first, stop bit low.
inline std::vector<int16_t> make_videodat_line(
    const std::vector<uint8_t>& bytes, const SyntheticLineConfig& config) {
  const double spb = 17734475.0 / 2000000.0;
  std::vector<int16_t> out(config.length, static_cast<int16_t>(config.low));
  if (config.with_sync) {
    for (size_t i = 0; i < config.sync_end && i < out.size(); ++i) {
      out[i] = static_cast<int16_t>(config.sync_tip);
    }
  }
  for (size_t i = 0; i < out.size(); ++i) {
    const double t = (static_cast<double>(i) + 0.5 - config.start_edge) / spb;
    if (t < 0.0) continue;
    const size_t bit = static_cast<size_t>(t);
    if (bit >= bytes.size() * 10) continue;
    const size_t position = bit % 10;
    bool level = false;
    if (position == 0) {
      level = true;
    } else if (position <= 8) {
      level = ((bytes[bit / 10] >> (position - 1)) & 1u) == 0;
    }
    out[i] = static_cast<int16_t>(level ? config.high : config.low);
  }
  return out;
}

// A recording whose data lines carry a byte stream, ten bytes a line, in
// field-line order; every other line is blanking.
class SyntheticVideoDatSource : public orc::IVideoDatFrameSource {
 public:
  SyntheticVideoDatSource(const std::vector<uint8_t>& stream,
                          std::vector<size_t> lines_per_field,
                          uint64_t minimum_frames,
                          SyntheticLineConfig config = {})
      : lines_(std::move(lines_per_field)), config_(config) {
    std::vector<uint8_t> padded = stream;
    padded.resize((padded.size() + 9) / 10 * 10, 0);
    const size_t data_lines = padded.size() / 10;
    const size_t per_frame = std::max<size_t>(1, 2 * lines_.size());
    frames_ = std::max<uint64_t>(minimum_frames,
                                 (data_lines + per_frame - 1) / per_frame);
    size_t next = 0;
    for (uint64_t frame = 0; frame < frames_; ++frame) {
      for (int field = 0; field < 2; ++field) {
        for (const size_t line : lines_) {
          std::vector<uint8_t> bytes(10, 0);
          if (next < data_lines) {
            bytes.assign(
                padded.begin() + static_cast<ptrdiff_t>(next * 10),
                padded.begin() + static_cast<ptrdiff_t>(next * 10 + 10));
          }
          ++next;
          data_[{frame, field, line}] = make_videodat_line(bytes, config_);
        }
      }
    }
    SyntheticLineConfig blank = config_;
    blank_ = make_videodat_line({}, blank);
  }

  uint64_t frame_count() const override { return frames_; }

  bool read_line(uint64_t frame, int field, size_t line,
                 std::vector<int16_t>& out) const override {
    if (frame >= frames_) return false;
    const auto it = data_.find({frame, field, line});
    out = (it == data_.end()) ? blank_ : it->second;
    return true;
  }

 private:
  struct Key {
    uint64_t frame;
    int field;
    size_t line;
    bool operator<(const Key& other) const {
      return std::tie(frame, field, line) <
             std::tie(other.frame, other.field, other.line);
    }
  };

  std::vector<size_t> lines_;
  SyntheticLineConfig config_;
  uint64_t frames_ = 0;
  std::map<Key, std::vector<int16_t>> data_;
  std::vector<int16_t> blank_;
};

}  // namespace orc_unit_test

#endif  // ORC_TESTS_VIDEODAT_TEST_SIGNAL_H
