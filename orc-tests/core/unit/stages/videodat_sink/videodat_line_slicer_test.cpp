/*
 * File:        videodat_line_slicer_test.cpp
 * Module:      orc-core-tests
 * Purpose:     Unit tests for the VideoDat line slicer
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_line_slicer.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "videodat_test_signal.h"

namespace orc_unit_test {
namespace {

using orc::VideoDatSlicerConfig;

const std::vector<uint8_t> kBytes = {0x56, 0xAE, 0x10, 0x05, 0x33,
                                     0xC1, 0x02, 0x99, 0x4D, 0xE7};

TEST(VideoDatLineSlicer, FindsTheStartEdgeAndLevels) {
  SyntheticLineConfig config;
  config.start_edge = 230.25;
  const auto line = make_videodat_line(kBytes, config);

  const auto phase = orc::videodat_line_phase(line.data(), line.size(),
                                              VideoDatSlicerConfig{});
  ASSERT_TRUE(phase.has_value());
  EXPECT_NEAR(phase->start_edge, config.start_edge, 0.5);
  EXPECT_NEAR(phase->low, config.low, 1.0);
  EXPECT_NEAR(phase->high, config.high, 1.0);
}

TEST(VideoDatLineSlicer, BlankLineHasNoPhase) {
  const std::vector<int16_t> line(1135, 256);
  EXPECT_FALSE(
      orc::videodat_line_phase(line.data(), line.size(), VideoDatSlicerConfig{})
          .has_value());
}

TEST(VideoDatLineSlicer, DataLineIsFullyFramed) {
  const auto line = make_videodat_line(kBytes, SyntheticLineConfig{});
  const VideoDatSlicerConfig config;
  const auto phase = orc::videodat_line_phase(line.data(), line.size(), config);
  ASSERT_TRUE(phase.has_value());
  EXPECT_EQ(
      orc::videodat_line_framing(line.data(), line.size(), *phase, config),
      20u);

  // Read half a byte late, start and stop bits land inside the data.
  orc::VideoDatLinePhase late = *phase;
  late.start_edge += 5.0 * config.samples_per_bit();
  EXPECT_LT(orc::videodat_line_framing(line.data(), line.size(), late, config),
            20u);
}

TEST(VideoDatLineSlicer, SlicesTheBytesBack) {
  const auto line = make_videodat_line(kBytes, SyntheticLineConfig{});
  const VideoDatSlicerConfig config;
  const auto phase = orc::videodat_line_phase(line.data(), line.size(), config);
  ASSERT_TRUE(phase.has_value());
  const auto sliced = orc::videodat_slice_features(
      orc::videodat_line_features(line.data(), line.size(), *phase, config));

  EXPECT_EQ(std::vector<uint8_t>(sliced.bytes.begin(), sliced.bytes.end()),
            kBytes);
  // Soft values carry the decision: positive where the wire was high, which
  // is a zero bit of the (inverted) byte.
  EXPECT_GT(sliced.soft[0], 0.0f);  // bit 0 of 0x56 is 0
  EXPECT_LT(sliced.soft[1], 0.0f);  // bit 1 of 0x56 is 1
}

TEST(VideoDatLineSlicer, PriorTimingIsRefinedByTheTransitions) {
  SyntheticLineConfig line_config;
  line_config.start_edge = 225.6;
  const auto line = make_videodat_line(kBytes, line_config);
  // A prior a quarter of a bit early is pulled onto the transitions; hard
  // steps sampled at whole samples place them to within about half a sample.
  const auto phase = orc::videodat_line_phase(line.data(), line.size(),
                                              VideoDatSlicerConfig{}, 223.4);
  ASSERT_TRUE(phase.has_value());
  EXPECT_NEAR(phase->start_edge, line_config.start_edge, 0.75);
}

TEST(VideoDatLineSlicer, FindsTheEndOfLineSync) {
  SyntheticLineConfig config;
  const auto line = make_videodat_line(kBytes, config);
  const auto end =
      orc::videodat_sync_end(line.data(), line.size(), VideoDatSlicerConfig{});
  ASSERT_TRUE(end.has_value());
  EXPECT_NEAR(*end, static_cast<double>(config.sync_end) - 0.5, 0.6);

  config.with_sync = false;
  const auto no_sync = make_videodat_line(kBytes, config);
  EXPECT_FALSE(orc::videodat_sync_end(no_sync.data(), no_sync.size(),
                                      VideoDatSlicerConfig{})
                   .has_value());
}

TEST(VideoDatLineSlicer, DecisionsBecomeInvertedBytesLeastSignificantFirst) {
  std::array<double, orc::kVideoDatBitsPerLine> decisions{};
  decisions.fill(-1.0);  // every wire bit low: every data bit reads 1
  const auto line = orc::videodat_line_from_decisions(decisions);
  for (const uint8_t byte : line.bytes) EXPECT_EQ(byte, 0xFF);

  decisions.fill(1.0);
  decisions[1] = -1.0;  // first data bit of byte 0 low
  const auto second = orc::videodat_line_from_decisions(decisions);
  EXPECT_EQ(second.bytes[0], 0x01);
  EXPECT_EQ(second.bytes[1], 0x00);
}

TEST(VideoDatLineSlicer, EqualiserFittedOnCleanLinesDecodesThem) {
  const VideoDatSlicerConfig config;
  std::vector<orc::VideoDatLineFeatures> lines;
  for (uint8_t seed = 0; seed < 20; ++seed) {
    std::vector<uint8_t> bytes(10);
    for (size_t i = 0; i < bytes.size(); ++i) {
      bytes[i] = static_cast<uint8_t>(seed * 37 + i * 101);
    }
    const auto line = make_videodat_line(bytes, SyntheticLineConfig{});
    const auto phase =
        orc::videodat_line_phase(line.data(), line.size(), config);
    ASSERT_TRUE(phase.has_value());
    lines.push_back(
        orc::videodat_line_features(line.data(), line.size(), *phase, config));
  }
  std::vector<const orc::VideoDatLineFeatures*> rows;
  for (const auto& line : lines) rows.push_back(&line);
  const auto equaliser = orc::videodat_fit_equaliser(rows);
  ASSERT_TRUE(equaliser.has_value());

  for (const auto& features : lines) {
    const auto plain = orc::videodat_slice_features(features);
    const auto equalised = orc::videodat_slice_features(features, &*equaliser);
    EXPECT_EQ(plain.bytes, equalised.bytes);
  }
}

TEST(VideoDatLineSlicer, EqualiserNeedsRows) {
  EXPECT_FALSE(orc::videodat_fit_equaliser({}).has_value());
}

}  // namespace
}  // namespace orc_unit_test
