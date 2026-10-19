/*
 * File:        videodat_file_assembler_test.cpp
 * Module:      orc-core-tests
 * Purpose:     Unit tests for VideoDat file reassembly and services
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_file_assembler.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "videodat_services.h"
#include "videodat_test_signal.h"

namespace orc_unit_test {
namespace {

using orc::VideoDatPacket;

// DOS time of 1993-07-08 16:43:38.
constexpr uint32_t kDosTime =
    (13u << 25) | (7u << 21) | (8u << 16) | (16u << 11) | (43u << 5) | 19u;

void put_le32(std::vector<uint8_t>& bytes, size_t offset, uint32_t value) {
  for (size_t i = 0; i < 4; ++i) {
    bytes[offset + i] = static_cast<uint8_t>(value >> (8 * i));
  }
}

std::vector<uint8_t> header_payload(const std::string& name, uint32_t size) {
  std::vector<uint8_t> payload(32, 0);
  payload[0] = 0x02;
  put_le32(payload, 5, size);
  put_le32(payload, 9, kDosTime);
  payload[13] = static_cast<uint8_t>(name.size());
  std::copy(name.begin(), name.end(), payload.begin() + 14);
  return payload;
}

std::vector<uint8_t> data_payload(size_t n, uint8_t seed) {
  std::vector<uint8_t> payload(n);
  payload[0] = 0x04;
  for (size_t i = 1; i < n; ++i) {
    payload[i] = static_cast<uint8_t>(seed + i);
  }
  return payload;
}

std::vector<uint8_t> end_payload() {
  std::vector<uint8_t> payload(8, 0);
  payload[0] = 0x03;
  return payload;
}

// A 300-byte file in a 32-byte header, a 255-byte and a 63-byte block, and an
// end mark, counters from `counter` on.
std::vector<VideoDatPacket> file_packets(uint8_t counter, size_t position,
                                         bool with_last_block = true) {
  std::vector<VideoDatPacket> out;
  out.push_back(received_packet(0x65, videodat_control(5), counter,
                                header_payload("TEST.BIN", 300), position));
  out.push_back(received_packet(0x65, videodat_control(8), counter + 1,
                                data_payload(256, 1), position + 100));
  if (with_last_block) {
    out.push_back(received_packet(0x65, videodat_control(6), counter + 2,
                                  data_payload(64, 50), position + 400));
  }
  out.push_back(received_packet(0x65, videodat_control(3), counter + 3,
                                end_payload(), position + 500));
  return out;
}

const orc::VideoDatOutputFile* find_output(
    const orc::VideoDatAssembly& assembly, const std::string& path) {
  for (const auto& output : assembly.outputs) {
    if (output.path == path) return &output;
  }
  return nullptr;
}

TEST(VideoDatFileAssembler, DosTimeDecodesAndRejectsImpossibleDates) {
  const auto t = orc::videodat_dos_time(kDosTime);
  ASSERT_TRUE(t.has_value());
  EXPECT_EQ(t->year, 1993);
  EXPECT_EQ(t->month, 7);
  EXPECT_EQ(t->day, 8);
  EXPECT_EQ(t->hour, 16);
  EXPECT_EQ(t->minute, 43);
  EXPECT_EQ(t->second, 38);
  EXPECT_FALSE(orc::videodat_dos_time(0).has_value());  // month 0
}

TEST(VideoDatFileAssembler, DateTimeRoundTripsThroughSeconds) {
  const orc::VideoDatDateTime t{1994, 1, 6, 20, 37, 19};
  EXPECT_EQ(
      orc::VideoDatDateTime::from_seconds_since_epoch(t.seconds_since_epoch()),
      t);
  const orc::VideoDatDateTime epoch{1970, 1, 1, 0, 0, 0};
  EXPECT_EQ(epoch.seconds_since_epoch(), 0);
  EXPECT_EQ(orc::videodat_format_date(t), "06.01.1994");
  EXPECT_EQ(orc::videodat_format_time(t), "20:37:19");
}

TEST(VideoDatFileAssembler, SafeNamesDecodeCodePage437AndDropPathCharacters) {
  const std::vector<uint8_t> umlaut = {'B', 0x81, 'R', 'O', '.', 'T', 'X', 'T'};
  EXPECT_EQ(orc::videodat_safe_name(umlaut.data(), umlaut.size()),
            "B\xC3\xBCRO.TXT");
  const std::string path = " ..A/B:C\x01. ";
  EXPECT_EQ(orc::videodat_safe_name(
                reinterpret_cast<const uint8_t*>(path.data()), path.size()),
            "A_B_C_");
  const std::string empty = " . ";
  EXPECT_EQ(orc::videodat_safe_name(
                reinterpret_cast<const uint8_t*>(empty.data()), empty.size()),
            "unbenannt");
}

TEST(VideoDatFileAssembler, CounterUnwrapsAcrossItsEightBits) {
  const auto u = orc::unwrap_videodat_counter({250, 252, 254, 0, 2, 1, 4});
  ASSERT_EQ(u.size(), 7u);
  EXPECT_EQ(u[3], 256);
  EXPECT_EQ(u[5], 257);
  EXPECT_EQ(u[6], 260);
}

TEST(VideoDatFileAssembler, AnOutlierNothingFollowsIsDropped) {
  // A counter far behind the section, with no packets after it to show a cut.
  const auto u = orc::unwrap_videodat_counter({10, 11, 12, 200});
  ASSERT_EQ(u.size(), 4u);
  EXPECT_EQ(u[2], 12);
  EXPECT_FALSE(u[3].has_value());
}

TEST(VideoDatFileAssembler, ACutOpensANewSection) {
  const auto u = orc::unwrap_videodat_counter({10, 11, 200, 201, 202});
  ASSERT_TRUE(u[2].has_value());
  EXPECT_GT(*u[2], 11 + 1000);
  EXPECT_EQ(*u[3], *u[2] + 1);
}

TEST(VideoDatFileAssembler, ReassemblesACompleteFile) {
  const auto assembly = orc::assemble_videodat_files(
      file_packets(20, 0), orc::VideoDatAssemblyOptions{});

  ASSERT_EQ(assembly.files.size(), 1u);
  const auto& file = assembly.files[0];
  EXPECT_EQ(file.stream, 0x65);
  EXPECT_EQ(file.name, "TEST.BIN");
  EXPECT_EQ(file.size, 300u);
  EXPECT_EQ(file.missing_blocks, 0u);
  EXPECT_EQ(file.blocks, 2u);
  ASSERT_TRUE(file.output.has_value());

  const auto& output = assembly.outputs[*file.output];
  EXPECT_EQ(output.path, "strom_65/TEST.BIN");
  ASSERT_EQ(output.data.size(), 300u);
  EXPECT_EQ(output.data[0], 2);     // first byte of block 0
  EXPECT_EQ(output.data[255], 51);  // first byte of block 1
  ASSERT_TRUE(output.modified.has_value());
  EXPECT_EQ(output.modified->year, 1993);
}

TEST(VideoDatFileAssembler, IncompleteFilesAreWrittenOnlyWhenAskedFor) {
  const auto packets = file_packets(20, 0, false);

  orc::VideoDatAssemblyOptions keep;
  const auto kept = orc::assemble_videodat_files(packets, keep);
  ASSERT_EQ(kept.files.size(), 1u);
  EXPECT_EQ(kept.files[0].missing_blocks, 1u);
  ASSERT_TRUE(kept.files[0].output.has_value());
  const auto& output = kept.outputs[*kept.files[0].output];
  EXPECT_EQ(output.path, "strom_65/TEST.BIN.unvollstaendig");
  EXPECT_EQ(output.data.size(), 300u);
  EXPECT_EQ(output.data[299], 0);

  orc::VideoDatAssemblyOptions drop;
  drop.keep_partial = false;
  const auto dropped = orc::assemble_videodat_files(packets, drop);
  ASSERT_EQ(dropped.files.size(), 1u);
  EXPECT_FALSE(dropped.files[0].output.has_value());
  EXPECT_TRUE(dropped.outputs.empty());
}

TEST(VideoDatFileAssembler, TransmissionsOfOneFileAreCombined) {
  auto first = file_packets(20, 0, false);
  auto second = file_packets(40, 2000);
  second.erase(second.begin() + 1);  // the first block lost this time
  first.insert(first.end(), second.begin(), second.end());

  const auto assembly =
      orc::assemble_videodat_files(first, orc::VideoDatAssemblyOptions{});
  ASSERT_EQ(assembly.files.size(), 1u);
  EXPECT_EQ(assembly.files[0].transmissions, 2u);
  EXPECT_EQ(assembly.files[0].missing_blocks, 0u);
  EXPECT_EQ(assembly.outputs[*assembly.files[0].output].path,
            "strom_65/TEST.BIN");
}

TEST(VideoDatFileAssembler, DataWithoutAHeaderIsKeptAsARun) {
  std::vector<VideoDatPacket> packets;
  packets.push_back(
      received_packet(0x11, videodat_control(4), 92, data_payload(16, 1), 0));
  packets.push_back(
      received_packet(0x11, videodat_control(4), 94, data_payload(16, 2), 100));
  packets.push_back(
      received_packet(0x11, videodat_control(3), 95, end_payload(), 200));

  const auto assembly =
      orc::assemble_videodat_files(packets, orc::VideoDatAssemblyOptions{});
  const auto* run =
      find_output(assembly, "strom_11/ohne_dateikopf_92.bin.unvollstaendig");
  ASSERT_NE(run, nullptr);
  // Blocks 92 and 94 with 93 filled in.
  EXPECT_EQ(run->data.size(), 45u);
  EXPECT_TRUE(std::any_of(
      assembly.log.begin(), assembly.log.end(), [](const std::string& line) {
        return line.find("end of file received") != std::string::npos;
      }));
}

TEST(VideoDatFileAssembler, OtherServicesAreCountedAndOptionallySaved) {
  std::vector<uint8_t> clock = {0x06, 94, 1, 6, 20, 37, 19, 0};
  std::vector<VideoDatPacket> packets = {
      received_packet(0x01, videodat_control(3), 1, clock, 0)};
  orc::VideoDatAssemblyOptions options;
  options.dump_other = true;
  const auto assembly = orc::assemble_videodat_files(packets, options);
  const auto* dump =
      find_output(assembly, "andere_dienste/strom_01_typ_06.bin");
  ASSERT_NE(dump, nullptr);
  EXPECT_EQ(dump->data, clock);
  EXPECT_NE(std::find(assembly.log.begin(), assembly.log.end(),
                      "Stream 01/0: other packet types 06:1"),
            assembly.log.end());
}

TEST(VideoDatFileAssembler, RecognisesPurchasedSoftware) {
  std::vector<uint8_t> data(40, 0);
  const std::string magic = "CHVDFILE";
  std::copy(magic.begin(), magic.end(), data.begin());
  data[8] = 4;
  put_le32(data, 15, 12345);
  const std::string name = "!GAME.EXE";
  std::copy(name.begin(), name.end(), data.begin() + 19);
  const auto header = orc::videodat_chvd_header(data);
  ASSERT_TRUE(header.has_value());
  EXPECT_EQ(header->length, 12345u);
  EXPECT_EQ(header->name, "GAME.EXE");
  EXPECT_FALSE(orc::videodat_chvd_header(std::vector<uint8_t>(40, 0)));
}

TEST(VideoDatServices, RunLengthDecoding) {
  const std::vector<uint8_t> coded = {0x00, 3, 0x12, 0xFF, 2, 0x34};
  EXPECT_EQ(orc::videodat_unrun(coded.data(), coded.size()),
            (std::vector<uint8_t>{0, 0, 0, 0x12, 0xFF, 0xFF, 0x34}));
}

TEST(VideoDatServices, ClockMapsBytesToBroadcastTime) {
  const orc::VideoDatDateTime start{1994, 1, 6, 20, 37, 19};
  const int64_t t0 = start.seconds_since_epoch();
  orc::VideoDatClock clock({{0, t0}, {7500, t0 + 5}, {15000, t0 + 10}});
  ASSERT_TRUE(clock.bytes_per_second().has_value());
  EXPECT_DOUBLE_EQ(*clock.bytes_per_second(), 1500.0);
  const auto at = clock.at(9000);
  ASSERT_TRUE(at.has_value());
  EXPECT_EQ(orc::videodat_format_time(*at), "20:37:25");
  EXPECT_TRUE(clock.gaps().empty());
}

TEST(VideoDatServices, ReportsClockAndBitLists) {
  std::vector<VideoDatPacket> packets;
  for (int k = 0; k < 4; ++k) {
    std::vector<uint8_t> tick = {
        0x06, 94, 1, 6, 20, 37, static_cast<uint8_t>(10 + 5 * k), 0};
    packets.push_back(received_packet(0x01, videodat_control(3), k, tick,
                                      static_cast<size_t>(7500 * k)));
  }
  // Bit list 11, block 0, one 32-byte segment: 00 n=32 run of zeros, then
  // overwrite nothing - all clear.
  std::vector<uint8_t> list(16, 0);
  list[0] = 0x00;  // type
  list[1] = 0x11;  // list
  list[5] = 1;     // one segment
  list[7] = 0xFF;  // 32 bytes of FF
  list[8] = 32;
  packets.push_back(received_packet(0x00, videodat_control(4), 9, list, 40000));

  const auto report = orc::analyse_videodat_services(packets);
  ASSERT_FALSE(report.lines.empty());
  EXPECT_EQ(report.lines[0], "CLOCK (stream 01, type 06)");
  EXPECT_NE(report.lines[1].find("4 timestamps, 06.01.1994 20:37:10"),
            std::string::npos);
  const auto bitlist = std::find_if(
      report.outputs.begin(), report.outputs.end(),
      [](const auto& o) { return o.path == "dienste/bitliste_11.bin"; });
  ASSERT_NE(bitlist, report.outputs.end());
  EXPECT_EQ(bitlist->data, std::vector<uint8_t>(32, 0xFF));
  EXPECT_EQ(report.outputs.back().path, "dienste/dienste.txt");
}

}  // namespace
}  // namespace orc_unit_test
