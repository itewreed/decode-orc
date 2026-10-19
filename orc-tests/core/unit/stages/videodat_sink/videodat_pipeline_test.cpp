/*
 * File:        videodat_pipeline_test.cpp
 * Module:      orc-core-tests
 * Purpose:     Unit tests for the VideoDat decoder from frames to files
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_pipeline.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "videodat_test_signal.h"

namespace orc_unit_test {
namespace {

// DOS time of 1994-01-06 20:30:00.
constexpr uint32_t kDosTime =
    (14u << 25) | (1u << 21) | (6u << 16) | (20u << 11) | (30u << 5);

// Three packets of a 200-byte file, sent twice, and its end mark.
std::vector<uint8_t> file_stream() {
  std::vector<uint8_t> header(32, 0);
  header[0] = 0x02;
  header[5] = 200;
  for (size_t i = 0; i < 4; ++i) {
    header[9 + i] = static_cast<uint8_t>(kDosTime >> (8 * i));
  }
  const std::string name = "DATA.TXT";
  header[13] = static_cast<uint8_t>(name.size());
  std::copy(name.begin(), name.end(), header.begin() + 14);

  std::vector<uint8_t> data(256, 0);
  data[0] = 0x04;
  for (size_t i = 1; i < data.size(); ++i) {
    data[i] = static_cast<uint8_t>('a' + i % 26);
  }
  std::vector<uint8_t> end(8, 0);
  end[0] = 0x03;

  std::vector<uint8_t> stream(30, 0);
  for (int copy = 0; copy < 2; ++copy) {
    const auto a = make_videodat_packet(0x65, videodat_control(5), 1, header);
    const auto b = make_videodat_packet(0x65, videodat_control(8), 2, data);
    const auto c = make_videodat_packet(0x65, videodat_control(3), 3, end);
    for (const auto* packet : {&a, &b, &c}) {
      stream.insert(stream.end(), packet->begin(), packet->end());
      stream.insert(stream.end(), 7, 0);
    }
  }
  return stream;
}

const orc::VideoDatOutputFile* find_output(
    const orc::VideoDatPipelineResult& result, const std::string& path) {
  for (const auto& output : result.outputs) {
    if (output.path == path) return &output;
  }
  return nullptr;
}

bool log_contains(const orc::VideoDatPipelineResult& result,
                  const std::string& text) {
  return std::any_of(result.log.begin(), result.log.end(),
                     [&](const std::string& line) {
                       return line.find(text) != std::string::npos;
                     });
}

TEST(VideoDatPipeline, DecodesAFileFromSynchronisedFrames) {
  const auto stream = file_stream();
  ASSERT_FALSE(holds_sync(
      std::vector<uint8_t>(stream.begin() + 32, stream.begin() + 40)));
  SyntheticVideoDatSource source(stream, {10, 11}, 40);

  orc::VideoDatPipelineOptions options;
  options.save_byte_stream = true;
  const auto result = orc::run_videodat_pipeline(source, options);

  ASSERT_FALSE(result.cancelled);
  EXPECT_TRUE(log_contains(result, "VideoDat lines (PAL): 11, 12, 324, 325"));
  EXPECT_TRUE(log_contains(result, "samples after the end of line sync"));
  EXPECT_EQ(result.packets, 6u);
  EXPECT_EQ(result.packets_ok, 6u);
  EXPECT_EQ(result.files, 1u);
  EXPECT_EQ(result.complete_files, 1u);

  const auto* file = find_output(result, "strom_65/DATA.TXT");
  ASSERT_NE(file, nullptr);
  ASSERT_EQ(file->data.size(), 200u);
  EXPECT_EQ(file->data[0], 'b');
  EXPECT_EQ(file->data[25], 'a');

  const auto* bytes = find_output(result, orc::kVideoDatByteStreamName);
  ASSERT_NE(bytes, nullptr);
  EXPECT_TRUE(std::equal(stream.begin(), stream.end(), bytes->data.begin()));
  EXPECT_NE(find_output(result, "dienste/dienste.txt"), nullptr);
}

TEST(VideoDatPipeline, DecodesFramesWithoutSync) {
  SyntheticLineConfig config;
  config.with_sync = false;
  SyntheticVideoDatSource source(file_stream(), {5, 6, 7}, 40, config);

  orc::VideoDatPipelineOptions options;
  options.analyse_services = false;
  const auto result = orc::run_videodat_pipeline(source, options);

  EXPECT_FALSE(log_contains(result, "end of line sync"));
  EXPECT_EQ(result.complete_files, 1u);
  EXPECT_NE(find_output(result, "strom_65/DATA.TXT"), nullptr);
  EXPECT_EQ(find_output(result, "dienste/dienste.txt"), nullptr);
}

TEST(VideoDatPipeline, ReportsARecordingWithoutVideoDat) {
  SyntheticVideoDatSource source({}, {}, 30);
  const auto result =
      orc::run_videodat_pipeline(source, orc::VideoDatPipelineOptions{});
  EXPECT_TRUE(log_contains(result, "No VideoDat lines found"));
  EXPECT_EQ(result.files, 0u);
}

TEST(VideoDatPipeline, StopsWhenCancelled) {
  SyntheticVideoDatSource source(file_stream(), {10, 11}, 40);
  const auto result = orc::run_videodat_pipeline(
      source, orc::VideoDatPipelineOptions{}, nullptr, [] { return true; });
  EXPECT_TRUE(result.cancelled);
  EXPECT_TRUE(result.outputs.empty());
}

TEST(VideoDatPipeline, ReportsProgress) {
  SyntheticVideoDatSource source(file_stream(), {10, 11}, 40);
  uint64_t last_done = 0;
  uint64_t last_total = 0;
  orc::run_videodat_pipeline(
      source, orc::VideoDatPipelineOptions{},
      [&](uint64_t done, uint64_t total, const std::string&) {
        last_done = done;
        last_total = total;
      });
  EXPECT_EQ(last_total, 40u);
  EXPECT_EQ(last_done, 40u);
}

}  // namespace
}  // namespace orc_unit_test
