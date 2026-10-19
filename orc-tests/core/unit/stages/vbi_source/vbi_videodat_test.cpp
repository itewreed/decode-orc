/*
 * File:        vbi_videodat_test.cpp
 * Module:      orc-core-tests
 * Purpose:     Unit tests for the VideoDat presets of the VBI source
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include <gtest/gtest.h>
#include <orc/stage/cvbs_signal_constants.h>

#include <cstdint>
#include <string>
#include <vector>

#include "vbi_output_frame.h"
#include "vbi_source_format.h"
#include "vbi_source_validation.h"
#include "vbi_start_edge_locator.h"
#include "vbi_teletext_service.h"

namespace orc {
namespace {

constexpr double kBitRate = 2000000.0;

// A VideoDat line as a capture records it: logic 0 until the first start
// bit, then ten framed bytes (start bit high, data bits inverted least
// significant first, stop bit low), then logic 0 again.
std::vector<double> videodat_record(const std::vector<uint8_t>& bytes,
                                    double sample_rate_hz, double start_edge,
                                    size_t length, double low, double high) {
  const double spb = sample_rate_hz / kBitRate;
  std::vector<double> out(length, low);
  for (size_t i = 0; i < length; ++i) {
    const double t = (static_cast<double>(i) - start_edge) / spb;
    if (t < 0.0) continue;
    const size_t bit = static_cast<size_t>(t);
    if (bit >= bytes.size() * 10) continue;
    const size_t byte = bit / 10;
    const size_t position = bit % 10;
    bool level = false;
    if (position == 0) {
      level = true;
    } else if (position <= 8) {
      level = ((bytes[byte] >> (position - 1)) & 1u) == 0;
    }
    out[i] = level ? high : low;
  }
  return out;
}

TEST(VBIVideoDat, CardPresetIsThePalContainerWithTheVideoDatService) {
  VBISourceFormat wst;
  VBISourceFormat videodat;
  std::string error;
  ASSERT_TRUE(
      expand_vbi_source_preset("bt8x8 card dump, 8-bit (WST)", wst, error));
  ASSERT_TRUE(expand_vbi_source_preset("bt8x8 card dump, 8-bit (VideoDat)",
                                       videodat, error))
      << error;

  EXPECT_EQ(videodat.tt_system, VBITeletextSystem::kVideoDat);
  EXPECT_EQ(videodat.tv_system, VBITVSystem::kPAL);
  EXPECT_DOUBLE_EQ(videodat.sample_rate_hz, wst.sample_rate_hz);
  EXPECT_EQ(videodat.line_length, wst.line_length);
  EXPECT_EQ(videodat.valid_samples, wst.valid_samples);
  EXPECT_EQ(videodat.field_lines, wst.field_lines);
  EXPECT_EQ(videodat.sample_format, VBISampleFormat::kU8);
  // The start edge moves between recordings, so it is searched for wider.
  EXPECT_GT(videodat.calibration.search_tolerance_samples,
            wst.calibration.search_tolerance_samples);
  EXPECT_TRUE(validate_vbi_source_config(videodat, std::nullopt).empty());
}

TEST(VBIVideoDat, CropPresetIsSixteenWholePalLinesAtFourFsc) {
  VBISourceFormat format;
  std::string error;
  ASSERT_TRUE(expand_vbi_source_preset(".tbc VBI crop, 16-bit (VideoDat)",
                                       format, error))
      << error;

  EXPECT_DOUBLE_EQ(format.sample_rate_hz, kPalSampleRate);
  EXPECT_EQ(format.line_length, 1135u);
  EXPECT_EQ(format.valid_samples, 1135u);
  EXPECT_EQ(format.sample_format, VBISampleFormat::kU16LE);
  EXPECT_EQ(format.field_lines, 16u);
  EXPECT_EQ(format.family, VBISourceFamily::kTBCDerived);
  EXPECT_DOUBLE_EQ(format.capture_offset_samples, 0.0);
  EXPECT_FALSE(format.capture_offset_is_auto);
  EXPECT_EQ(format.bytes_per_frame(), 2u * 16u * 1135u * 2u);
}

TEST(VBIVideoDat, EveryRecordOfAFieldIsPlaced) {
  EXPECT_EQ(standard_teletext_lines_per_field(VBITVSystem::kPAL,
                                              VBITeletextSystem::kVideoDat),
            16u);
}

TEST(VBIVideoDat, ServiceIsOneHundredBitsAtTwoMegabitsTimedByTheStartEdge) {
  VBITeletextService service;
  std::string error;
  ASSERT_TRUE(vbi_teletext_service(
      VBITVSystem::kPAL, VBITeletextSystem::kVideoDat, service, error))
      << error;

  EXPECT_DOUBLE_EQ(service.bit_rate_hz, kBitRate);
  EXPECT_EQ(service.transmitted_bits(), 100u);
  EXPECT_EQ(service.peak_reference_bits, 100u);
  EXPECT_EQ(service.timing_reference, VBITimingReference::kStartBitEdge);
  EXPECT_GT(service.guard_bits, 1u);
  EXPECT_DOUBLE_EQ(service.t_offset_ns, 12200.0);
}

TEST(VBIVideoDat, ServiceIsRefusedOutsidePal) {
  VBITeletextService service;
  std::string error;
  EXPECT_FALSE(vbi_teletext_service(
      VBITVSystem::kNTSC, VBITeletextSystem::kVideoDat, service, error));
  EXPECT_FALSE(error.empty());
}

TEST(VBIVideoDat, OutputLevelsAreBlankingAndAboveForPal) {
  VBIOutputFrame frame;
  std::string error;
  ASSERT_TRUE(make_vbi_output_frame(VBITVSystem::kPAL,
                                    VBITeletextSystem::kVideoDat, frame, error))
      << error;
  EXPECT_EQ(static_cast<int32_t>(frame.levels.logic0), kPalBlanking);
  EXPECT_GT(frame.levels.data_amplitude(), 300);
}

TEST(VBIVideoDat, ServiceNameIsVideoDat) {
  EXPECT_EQ(to_string(VBITeletextSystem::kVideoDat), "VideoDat");
}

TEST(VBIVideoDat, FramingCountsStartAndStopBits) {
  const double rate = kPalSampleRate;
  const double spb = rate / kBitRate;
  const std::vector<uint8_t> bytes = {0x00, 0xFF, 0x56, 0xAE, 0x12,
                                      0x34, 0x80, 0x01, 0x7E, 0xC3};
  const std::vector<double> record =
      videodat_record(bytes, rate, 210.3, 1135, 16000.0, 40000.0);

  EXPECT_EQ(vbi_line_framing_bits(record, 210.3, 28000.0, spb, 10, 10), 20u);
  // Half a byte late, every start and stop bit reads inside the data.
  EXPECT_LT(
      vbi_line_framing_bits(record, 210.3 + 5.0 * spb, 28000.0, spb, 10, 10),
      20u);
}

TEST(VBIVideoDat, StartEdgeIsFoundToAFractionOfASample) {
  VBITeletextService service;
  std::string error;
  ASSERT_TRUE(vbi_teletext_service(
      VBITVSystem::kPAL, VBITeletextSystem::kVideoDat, service, error));
  const double rate = kPalSampleRate;
  const std::vector<uint8_t> bytes = {0x56, 0xAE, 0x10, 0x05, 0x33,
                                      0xC1, 0x02, 0x99, 0x4D, 0xE7};
  const double edge = 216.37;
  const std::vector<double> record =
      videodat_record(bytes, rate, edge, 1135, 16384.0, 43000.0);

  VBIStartEdgeConfig config;
  config.minimum_swing_counts =
      vbi_start_edge_minimum_swing(VBISampleFormat::kU16LE);
  const VBICRIDetection detection = detect_vbi_start_edge_position(
      record, service, rate, VBICRISearchWindow{180.0, 260.0}, config);

  ASSERT_TRUE(detection.accepted);
  EXPECT_NEAR(detection.anchor_position_samples, edge, 0.5);
  EXPECT_DOUBLE_EQ(detection.peak_correlation, 1.0);
}

TEST(VBIVideoDat, FlatRecordHasNoStartEdge) {
  VBITeletextService service;
  std::string error;
  ASSERT_TRUE(vbi_teletext_service(
      VBITVSystem::kPAL, VBITeletextSystem::kVideoDat, service, error));
  const std::vector<double> record(1135, 16384.0);
  VBIStartEdgeConfig config;
  config.minimum_swing_counts =
      vbi_start_edge_minimum_swing(VBISampleFormat::kU16LE);
  const VBICRIDetection detection =
      detect_vbi_start_edge_position(record, service, kPalSampleRate,
                                     VBICRISearchWindow{180.0, 260.0}, config);
  EXPECT_FALSE(detection.accepted);
}

TEST(VBIVideoDat, MinimumSwingFollowsTheWordSize) {
  EXPECT_LT(vbi_start_edge_minimum_swing(VBISampleFormat::kU8),
            vbi_start_edge_minimum_swing(VBISampleFormat::kU16LE));
}

}  // namespace
}  // namespace orc
