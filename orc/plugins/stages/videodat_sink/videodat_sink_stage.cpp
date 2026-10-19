/*
 * File:        videodat_sink_stage.cpp
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     VideoDat Sink Stage - recovers Channel VideoDat transmissions
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_sink_stage.h"

#include <orc/support/logging.h>

#include <memory>
#include <stdexcept>
#include <variant>

#include "videodat_sink_deps.h"

namespace orc {

namespace {

constexpr const char* kOutputPath = "output_path";
constexpr const char* kWritePartial = "write_partial";
constexpr const char* kDumpOther = "dump_other";
constexpr const char* kAnalyseServices = "analyse_services";
constexpr const char* kSaveByteStream = "save_byte_stream";
constexpr const char* kUseSync = "use_sync";
constexpr const char* kUseEqualiser = "use_equaliser";
constexpr const char* kTrackLines = "track_lines";

bool get_bool_or(const std::map<std::string, ParameterValue>& parameters,
                 const std::string& name, bool fallback) {
  const auto it = parameters.find(name);
  if (it == parameters.end() || !std::holds_alternative<bool>(it->second)) {
    return fallback;
  }
  return std::get<bool>(it->second);
}

ParameterDescriptor bool_parameter(const std::string& name,
                                   const std::string& display_name,
                                   const std::string& description,
                                   bool default_value) {
  ParameterDescriptor desc;
  desc.name = name;
  desc.display_name = display_name;
  desc.description = description;
  desc.type = ParameterType::BOOL;
  desc.constraints.default_value = default_value;
  return desc;
}

}  // namespace

VideoDatSinkStage::VideoDatSinkStage() {
  set_configuration_status(orc::ConfigurationStatus::Red);
}

NodeTypeInfo VideoDatSinkStage::get_node_type_info() const {
  return NodeTypeInfo{
      NodeType::SINK,
      "videodat_sink",
      "VideoDat Sink",
      "Decodes Pro 7 Channel VideoDat from the VBI and writes the files it "
      "carried, with a report of its other services",
      1,
      1,  // One input
      0,
      0,  // No outputs (sink)
      VideoFormatCompatibility::PAL_ONLY};
}

std::vector<ArtifactPtr> VideoDatSinkStage::execute(
    const std::vector<ArtifactPtr>& inputs,
    const std::map<std::string, ParameterValue>& parameters,
    ObservationContext& observation_context) {
  // Sink stages do their work in trigger().
  (void)inputs;
  (void)parameters;
  (void)observation_context;
  return {};
}

std::vector<ParameterDescriptor> VideoDatSinkStage::get_parameter_descriptors(
    VideoSystem project_format, SourceType source_type) const {
  (void)project_format;
  (void)source_type;
  std::vector<ParameterDescriptor> descriptors;
  {
    ParameterDescriptor desc;
    desc.name = kOutputPath;
    desc.display_name = "Report File";
    desc.description =
        "Where the decoding report is written. The recovered files go into a "
        "folder next to it with the same name, without the extension.";
    desc.type = ParameterType::FILE_PATH;
    desc.constraints.required = true;
    desc.constraints.default_value = std::string("");
    desc.file_extension_hint = ".txt";
    desc.output_path = true;
    descriptors.push_back(desc);
  }
  descriptors.push_back(bool_parameter(
      kWritePartial, "Write Incomplete Files",
      "Also write files that were not received completely, and runs of data "
      "whose file header was missed, with the missing blocks filled with "
      "zeros. Their names end in .unvollstaendig.",
      true));
  descriptors.push_back(bool_parameter(
      kAnalyseServices, "Analyse Other Services",
      "Report the broadcast clock, the transmission schedule and the enabling "
      "bit lists, and save the encrypted account packets raw, in the "
      "dienste folder.",
      true));
  descriptors.push_back(bool_parameter(
      kDumpOther, "Save Other Packets",
      "Save the packets of every service other than file transfer raw, by "
      "stream and type, in the andere_dienste folder.",
      false));
  descriptors.push_back(bool_parameter(
      kSaveByteStream, "Save Byte Stream",
      "Also save the recovered byte stream (ten bytes per data line) as "
      "videodat_bytes.bin, for decoding again without the video.",
      false));
  descriptors.push_back(bool_parameter(
      kUseSync, "Use Line Sync",
      "On input that carries line sync (a .tbc through TBC Source), time each "
      "line from the end of its sync pulse, so lines with a damaged start "
      "edge are still read. Input without sync is timed from the start edge "
      "alone either way.",
      true));
  descriptors.push_back(bool_parameter(
      kUseEqualiser, "Train Equaliser",
      "Learn a linear equaliser and a decision threshold from the recording's "
      "own error-free lines. Recovers markedly more packets from worn tapes.",
      true));
  descriptors.push_back(bool_parameter(
      kTrackLines, "Track Data Lines",
      "On input without line sync, follow data lines that move within the "
      "field from frame to frame, as they do at the start of many card "
      "captures.",
      true));
  return descriptors;
}

std::map<std::string, ParameterValue> VideoDatSinkStage::get_parameters()
    const {
  return parameters_;
}

bool VideoDatSinkStage::set_parameters(
    const std::map<std::string, ParameterValue>& params) {
  parameters_ = params;
  const auto it = params.find(kOutputPath);
  const bool has_path = it != params.end() &&
                        std::holds_alternative<std::string>(it->second) &&
                        !std::get<std::string>(it->second).empty();
  set_configuration_status(has_path ? orc::ConfigurationStatus::Green
                                    : orc::ConfigurationStatus::Red);
  return true;
}

std::string VideoDatSinkStage::output_directory_for(
    const std::string& report_path) {
  const size_t slash = report_path.find_last_of("/\\");
  const size_t dot = report_path.rfind('.');
  const size_t name_start = (slash == std::string::npos) ? 0 : slash + 1;
  if (dot == std::string::npos || dot <= name_start) {
    return report_path + "_files";
  }
  return report_path.substr(0, dot);
}

VideoDatExportOptions VideoDatSinkStage::parse_options(
    const std::map<std::string, ParameterValue>& parameters) {
  const auto it = parameters.find(kOutputPath);
  if (it == parameters.end() ||
      !std::holds_alternative<std::string>(it->second) ||
      std::get<std::string>(it->second).empty()) {
    throw std::runtime_error("output_path parameter is required");
  }
  VideoDatExportOptions options;
  options.report_path = std::get<std::string>(it->second);
  options.output_directory = output_directory_for(options.report_path);
  VideoDatPipelineOptions& pipeline = options.pipeline;
  pipeline.assembly.keep_partial = get_bool_or(parameters, kWritePartial, true);
  pipeline.assembly.dump_other = get_bool_or(parameters, kDumpOther, false);
  pipeline.analyse_services = get_bool_or(parameters, kAnalyseServices, true);
  pipeline.save_byte_stream = get_bool_or(parameters, kSaveByteStream, false);
  pipeline.decoder.use_sync = get_bool_or(parameters, kUseSync, true);
  pipeline.decoder.use_equaliser = get_bool_or(parameters, kUseEqualiser, true);
  pipeline.decoder.track_lines = get_bool_or(parameters, kTrackLines, true);
  return options;
}

bool VideoDatSinkStage::trigger(
    const std::vector<ArtifactPtr>& inputs,
    const std::map<std::string, ParameterValue>& parameters,
    IObservationContext& observation_context) {
  (void)observation_context;
  is_processing_.store(true);
  cancel_requested_.store(false);
  try {
    if (inputs.empty()) {
      throw std::runtime_error(
          "VideoDat sink requires one input (VideoFrameRepresentation)");
    }
    const auto vfr =
        std::dynamic_pointer_cast<VideoFrameRepresentation>(inputs[0]);
    if (!vfr) {
      throw std::runtime_error("Input must be a VideoFrameRepresentation");
    }
    const VideoDatExportOptions options = parse_options(parameters);

    std::shared_ptr<IVideoDatSinkStageDeps> deps = deps_override_;
    if (!deps) deps = std::make_shared<VideoDatSinkStageDeps>();
    deps->init(progress_callback_, &cancel_requested_);

    const VideoDatExportResult result =
        deps->export_videodat(vfr.get(), options);
    is_processing_.store(false);
    if (!result.success) {
      ORC_LOG_ERROR("VideoDat sink error: {}", result.message.empty()
                                                   ? "Failed to decode VideoDat"
                                                   : result.message);
      return false;
    }
    ORC_LOG_INFO(
        "VideoDat sink: {} good packets, {} files ({} complete) written to {}",
        result.packets_ok, result.files, result.complete_files,
        options.output_directory);
    return true;
  } catch (const std::exception& e) {
    ORC_LOG_ERROR("VideoDat sink error: {}", e.what());
    is_processing_.store(false);
    return false;
  }
}

std::string VideoDatSinkStage::get_trigger_status() const {
  return is_processing_.load() ? "Decoding VideoDat..." : "Idle";
}

}  // namespace orc
