/*
 * File:        videodat_packet_test.cpp
 * Module:      orc-core-tests
 * Purpose:     Unit tests for VideoDat packet framing and repair
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_packet.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "videodat_test_signal.h"

namespace orc_unit_test {
namespace {

std::vector<uint8_t> pattern(size_t n, uint8_t seed) {
  std::vector<uint8_t> out(n);
  for (size_t i = 0; i < n; ++i) {
    out[i] = static_cast<uint8_t>(seed + i * 3);
  }
  return out;
}

// A byte stream with soft values that agree with its bytes, one unit strong.
orc::VideoDatByteStream with_soft(const std::vector<uint8_t>& data) {
  orc::VideoDatByteStream stream;
  stream.data = data;
  for (const uint8_t byte : data) {
    for (int bit = 0; bit < 8; ++bit) {
      // Bytes are inverted on the wire: a 0 bit was high, a positive decision.
      const bool one = ((byte >> bit) & 1u) != 0;
      stream.soft.push_back(one ? -64 : 64);
    }
  }
  return stream;
}

TEST(VideoDatPacket, LengthFollowsTheSizeField) {
  EXPECT_EQ(orc::videodat_packet_length(0x03), 8u + 11u);
  EXPECT_EQ(orc::videodat_packet_length(0x88), 256u + 11u);
}

TEST(VideoDatPacket, DescramblesWhatWasScrambled) {
  const std::vector<uint8_t> payload = pattern(32, 5);
  const auto raw = make_videodat_packet(0x65, videodat_control(5), 7, payload);
  std::vector<uint8_t> out;
  ASSERT_TRUE(orc::videodat_descramble(raw, out));
  EXPECT_EQ(out, payload);
  EXPECT_TRUE(orc::videodat_checksum_ok(raw));
}

TEST(VideoDatPacket, ShortPacketsAreNotPermuted) {
  const std::vector<uint8_t> payload = {0x03, 0x11, 0x22, 0x33};
  const auto raw = make_videodat_packet(0x10, videodat_control(2), 1, payload);
  std::vector<uint8_t> out;
  ASSERT_TRUE(orc::videodat_descramble(raw, out));
  EXPECT_EQ(out, payload);
}

TEST(VideoDatPacket, ADamagedByteFailsTheChecksum) {
  auto raw =
      make_videodat_packet(0x65, videodat_control(4), 9, pattern(16, 40));
  raw[12] ^= 0x10;
  std::vector<uint8_t> out;
  EXPECT_FALSE(orc::videodat_descramble(raw, out));
  EXPECT_FALSE(orc::videodat_checksum_ok(raw));
}

TEST(VideoDatPacket, FindsEveryPacketInTheStream) {
  const auto a =
      make_videodat_packet(0x65, videodat_control(4), 1, pattern(16, 1));
  const auto b =
      make_videodat_packet(0x10, videodat_control(3), 2, pattern(8, 100));
  std::vector<uint8_t> stream(7, 0);
  stream.insert(stream.end(), a.begin(), a.end());
  stream.insert(stream.end(), 3, 0);
  stream.insert(stream.end(), b.begin(), b.end());
  ASSERT_FALSE(holds_sync(std::vector<uint8_t>(a.begin() + 2, a.end())));

  const auto packets = orc::find_videodat_packets(stream);
  ASSERT_EQ(packets.size(), 2u);
  EXPECT_EQ(packets[0].position, 7u);
  EXPECT_TRUE(packets[0].ok);
  EXPECT_EQ(packets[0].stream(), 0x65);
  EXPECT_EQ(packets[0].counter(), 1);
  EXPECT_EQ(packets[0].payload, pattern(16, 1));
  EXPECT_EQ(packets[0].recovery, orc::VideoDatRecovery::kReceived);
  EXPECT_EQ(packets[1].position, 7u + a.size() + 3u);
  EXPECT_EQ(packets[1].stream(), 0x10);
}

TEST(VideoDatPacket, EscapedSyncWordsAreUnstuffed) {
  // A packet whose scrambled payload holds the sync word, sent with the escape
  // byte after it.
  std::vector<uint8_t> raw = {0x56, 0xAE, 0x65, videodat_control(3), 4, 0x07,
                              0x00, 0x00, 0x58};
  const std::vector<uint8_t> scrambled = {0x01, 0x56, 0xAE, 0x02,
                                          0x03, 0x04, 0x05, 0x06};
  raw.insert(raw.end(), scrambled.begin(), scrambled.end());
  raw.push_back(0);
  raw.push_back(0);
  std::vector<uint8_t> clear;
  orc::videodat_descramble(raw, clear);
  uint32_t sum = 0x65u + videodat_control(3) + 4u;
  for (const uint8_t value : clear) sum += value;
  raw[17] = static_cast<uint8_t>(sum & 0xFF);
  raw[18] = static_cast<uint8_t>(sum >> 8);
  ASSERT_TRUE(orc::videodat_checksum_ok(raw));

  std::vector<uint8_t> sent(raw.begin(), raw.begin() + 12);
  sent.push_back(0xFF);
  sent.insert(sent.end(), raw.begin() + 12, raw.end());
  const auto packets = orc::find_videodat_packets(sent);
  ASSERT_FALSE(packets.empty());
  EXPECT_TRUE(packets[0].ok);
  EXPECT_EQ(packets[0].recovery, orc::VideoDatRecovery::kUnstuffed);
  EXPECT_EQ(packets[0].raw, raw);
}

TEST(VideoDatPacket, SoftValuesSurviveQuantisationWithTheirSign) {
  EXPECT_EQ(orc::videodat_quantise_soft(-0.0001), -1);
  EXPECT_EQ(orc::videodat_quantise_soft(0.0), 0);
  EXPECT_EQ(orc::videodat_quantise_soft(1.0), 64);
  EXPECT_EQ(orc::videodat_quantise_soft(10.0), 127);
  EXPECT_EQ(orc::videodat_quantise_soft(-10.0), -127);
}

TEST(VideoDatPacket, CopiesRepairByTheirSummedSoftValues) {
  const auto good =
      make_videodat_packet(0x65, videodat_control(4), 3, pattern(16, 9));
  std::vector<uint8_t> data(good);
  data.insert(data.end(), 5, 0);
  data.insert(data.end(), good.begin(), good.end());
  orc::VideoDatByteStream stream = with_soft(data);

  // Damage one bit of the payload in each copy, weakly, at different places.
  const size_t second = good.size() + 5;
  const auto flip = [&](size_t byte, int bit) {
    stream.data[byte] ^= static_cast<uint8_t>(1u << bit);
    int8_t& soft = stream.soft[byte * 8 + static_cast<size_t>(bit)];
    soft = static_cast<int8_t>(soft > 0 ? -8 : 8);
  };
  flip(12, 2);
  flip(second + 15, 5);

  auto packets = orc::find_videodat_packets(stream);
  ASSERT_EQ(packets.size(), 2u);
  EXPECT_FALSE(packets[0].ok);
  EXPECT_FALSE(packets[1].ok);

  const auto counts = orc::repair_videodat_packets(packets);
  EXPECT_EQ(counts.soft, 1u);
  for (const auto& packet : packets) {
    EXPECT_TRUE(packet.ok);
    EXPECT_EQ(packet.recovery, orc::VideoDatRecovery::kSoft);
    EXPECT_EQ(packet.raw, good);
    EXPECT_FALSE(packet.unrepaired());
  }
}

TEST(VideoDatPacket, CopiesRepairByCombiningBytesWithoutSoftValues) {
  const auto good =
      make_videodat_packet(0x11, videodat_control(4), 3, pattern(16, 70));
  auto first = good;
  auto second = good;
  first[11] ^= 0x40;
  second[20] ^= 0x02;
  std::vector<uint8_t> data(first);
  data.insert(data.end(), 4, 0);
  data.insert(data.end(), second.begin(), second.end());

  auto packets = orc::find_videodat_packets(data);
  ASSERT_EQ(packets.size(), 2u);
  const auto counts = orc::repair_videodat_packets(packets);
  EXPECT_EQ(counts.copies, 1u);
  EXPECT_TRUE(packets[0].ok);
  EXPECT_EQ(packets[0].raw, good);
  EXPECT_EQ(packets[0].recovery, orc::VideoDatRecovery::kCopies);
}

TEST(VideoDatPacket, ASingleDamagedCopyIsNotRepairedOnSuspicion) {
  auto raw =
      make_videodat_packet(0x11, videodat_control(4), 3, pattern(16, 70));
  raw[14] ^= 0x01;
  auto packets = orc::find_videodat_packets(raw);
  ASSERT_EQ(packets.size(), 1u);
  const auto counts = orc::repair_videodat_packets(packets);
  EXPECT_EQ(counts.soft + counts.copies, 0u);
  EXPECT_FALSE(packets[0].ok);
}

}  // namespace
}  // namespace orc_unit_test
