/*
 * File:        videodat_sink_stage_test.cpp
 * Module:      orc-core-tests
 * Purpose:     Unit tests for VideoDatSinkStage parameters and trigger seam
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_sink_stage.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "../../include/observation_context_interface_mock.h"
#include "../../include/video_frame_representation_artifact_mock.h"
#include "videodat_sink_deps_interface.h"

namespace orc_unit_test {
namespace {

using testing::_;
using testing::NiceMock;
using testing::Return;
using testing::StrictMock;

class MockVideoDatSinkStageDeps : public orc::IVideoDatSinkStageDeps {
 public:
  MOCK_METHOD(void, init,
              (orc::TriggerProgressCallback progress_callback,
               std::atomic<bool>* cancel_requested),
              (override));
  MOCK_METHOD(orc::VideoDatExportResult, export_videodat,
              (const orc::VideoFrameRepresentation* representation,
               const orc::VideoDatExportOptions& options),
              (override));
};

const orc::ParameterDescriptor* find_descriptor(
    const std::vector<orc::ParameterDescriptor>& descriptors,
    const std::string& name) {
  const auto it = std::find_if(
      descriptors.begin(), descriptors.end(),
      [&](const orc::ParameterDescriptor& d) { return d.name == name; });
  return it == descriptors.end() ? nullptr : &*it;
}

TEST(VideoDatSinkStage, IsAPalOnlySink) {
  orc::VideoDatSinkStage stage;
  const auto info = stage.get_node_type_info();
  EXPECT_EQ(info.type, orc::NodeType::SINK);
  EXPECT_EQ(info.stage_name, "videodat_sink");
  EXPECT_EQ(info.compatible_formats, orc::VideoFormatCompatibility::PAL_ONLY);
  EXPECT_EQ(stage.required_input_count(), 1u);
  EXPECT_EQ(stage.output_count(), 0u);
}

TEST(VideoDatSinkStage, DescribesItsParameters) {
  orc::VideoDatSinkStage stage;
  const auto descriptors = stage.get_parameter_descriptors();

  const auto* output = find_descriptor(descriptors, "output_path");
  ASSERT_NE(output, nullptr);
  EXPECT_EQ(output->type, orc::ParameterType::FILE_PATH);
  EXPECT_TRUE(output->constraints.required);
  EXPECT_TRUE(output->output_path);

  for (const char* name :
       {"write_partial", "analyse_services", "dump_other", "save_byte_stream",
        "use_sync", "use_equaliser", "track_lines"}) {
    const auto* desc = find_descriptor(descriptors, name);
    ASSERT_NE(desc, nullptr) << name;
    EXPECT_EQ(desc->type, orc::ParameterType::BOOL) << name;
    ASSERT_TRUE(desc->constraints.default_value.has_value()) << name;
  }
}

TEST(VideoDatSinkStage, ConfigurationStatusFollowsTheReportPath) {
  orc::VideoDatSinkStage stage;
  EXPECT_EQ(stage.get_configuration_status(), orc::ConfigurationStatus::Red);
  stage.set_parameters({{"output_path", std::string("tape.txt")}});
  EXPECT_EQ(stage.get_configuration_status(), orc::ConfigurationStatus::Green);
  stage.set_parameters({{"output_path", std::string("")}});
  EXPECT_EQ(stage.get_configuration_status(), orc::ConfigurationStatus::Red);
}

TEST(VideoDatSinkStage, FilesGoNextToTheReport) {
  EXPECT_EQ(orc::VideoDatSinkStage::output_directory_for("/a/tape1.txt"),
            "/a/tape1");
  EXPECT_EQ(orc::VideoDatSinkStage::output_directory_for("/a.b/report"),
            "/a.b/report_files");
  EXPECT_EQ(orc::VideoDatSinkStage::output_directory_for("C:\\v\\t.log"),
            "C:\\v\\t");
}

TEST(VideoDatSinkStage, OptionsFollowTheParameters) {
  const auto defaults = orc::VideoDatSinkStage::parse_options(
      {{"output_path", std::string("out/x.txt")}});
  EXPECT_EQ(defaults.report_path, "out/x.txt");
  EXPECT_EQ(defaults.output_directory, "out/x");
  EXPECT_TRUE(defaults.pipeline.assembly.keep_partial);
  EXPECT_FALSE(defaults.pipeline.assembly.dump_other);
  EXPECT_TRUE(defaults.pipeline.analyse_services);
  EXPECT_FALSE(defaults.pipeline.save_byte_stream);
  EXPECT_TRUE(defaults.pipeline.decoder.use_sync);
  EXPECT_TRUE(defaults.pipeline.decoder.use_equaliser);
  EXPECT_TRUE(defaults.pipeline.decoder.track_lines);

  const auto changed = orc::VideoDatSinkStage::parse_options(
      {{"output_path", std::string("x.txt")},
       {"write_partial", false},
       {"dump_other", true},
       {"use_equaliser", false}});
  EXPECT_FALSE(changed.pipeline.assembly.keep_partial);
  EXPECT_TRUE(changed.pipeline.assembly.dump_other);
  EXPECT_FALSE(changed.pipeline.decoder.use_equaliser);

  EXPECT_THROW(orc::VideoDatSinkStage::parse_options({}), std::runtime_error);
}

TEST(VideoDatSinkStage, TriggerFailsWithoutInput) {
  orc::VideoDatSinkStage stage;
  MockObservationContext observation_context;
  EXPECT_FALSE(stage.trigger({}, {{"output_path", std::string("x.txt")}},
                             observation_context));
  EXPECT_FALSE(stage.is_trigger_in_progress());
  EXPECT_EQ(stage.get_trigger_status(), "Idle");
}

TEST(VideoDatSinkStage, TriggerUsesTheDepsSeam) {
  orc::VideoDatSinkStage stage;
  auto deps = std::make_shared<StrictMock<MockVideoDatSinkStageDeps>>();
  stage.set_deps_override(deps);
  NiceMock<MockObservationContext> observation_context;
  auto vfr = std::make_shared<NiceMock<MockVideoFrameRepresentationArtifact>>();

  orc::VideoDatExportOptions captured;
  orc::VideoDatExportResult result;
  result.success = true;
  result.files = 3;
  EXPECT_CALL(*deps, init(_, _));
  EXPECT_CALL(*deps, export_videodat(vfr.get(), _))
      .WillOnce(testing::DoAll(testing::SaveArg<1>(&captured), Return(result)));

  EXPECT_TRUE(stage.trigger(
      {vfr},
      {{"output_path", std::string("tape.txt")}, {"save_byte_stream", true}},
      observation_context));
  EXPECT_EQ(captured.report_path, "tape.txt");
  EXPECT_EQ(captured.output_directory, "tape");
  EXPECT_TRUE(captured.pipeline.save_byte_stream);
  EXPECT_FALSE(stage.is_trigger_in_progress());
}

TEST(VideoDatSinkStage, TriggerReportsAFailedExport) {
  orc::VideoDatSinkStage stage;
  auto deps = std::make_shared<StrictMock<MockVideoDatSinkStageDeps>>();
  stage.set_deps_override(deps);
  NiceMock<MockObservationContext> observation_context;
  auto vfr = std::make_shared<NiceMock<MockVideoFrameRepresentationArtifact>>();

  orc::VideoDatExportResult result;
  result.message = "not PAL";
  EXPECT_CALL(*deps, init(_, _));
  EXPECT_CALL(*deps, export_videodat(vfr.get(), _)).WillOnce(Return(result));

  EXPECT_FALSE(stage.trigger({vfr}, {{"output_path", std::string("t.txt")}},
                             observation_context));
  EXPECT_EQ(stage.get_trigger_status(), "Idle");
}

}  // namespace
}  // namespace orc_unit_test
