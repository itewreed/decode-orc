/*
 * File:        videodat_sink_stage.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     VideoDat Sink Stage - recovers Channel VideoDat transmissions
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_SINK_STAGE_H
#define ORC_VIDEODAT_SINK_STAGE_H

#include <orc/plugin/orc_stage_runtime.h>
#include <orc/stage/node_type.h>
#include <orc/stage/params/stage_parameter.h>
#include <orc/stage/triggerable_stage.h>
#include <orc/stage/video_frame_representation.h>

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "videodat_sink_deps_interface.h"

namespace orc {

/**
 * @brief VideoDat Sink Stage
 *
 * Decodes Channel VideoDat, the data service Pro 7 broadcast in the VBI from
 * 1993 to 1995, and writes the files it carried together with a report of the
 * other services (broadcast clock, transmission schedule, enabling bit
 * lists, encrypted account data).
 *
 * The input is any PAL frame representation whose VBI holds the data lines:
 * frames from a decoder's .tbc (through TBC Source, line sync included) or
 * from VBI Source reading a bt8x8 card dump or a .tbc VBI crop with one of
 * its VideoDat presets.
 *
 * Parameters:
 * - output_path: the report; the files are written to a folder named after
 *   it, without its extension
 * - write_partial, dump_other, analyse_services, save_byte_stream: what is
 *   written
 * - use_sync, use_equaliser, track_lines: how lines are read
 */
class VideoDatSinkStage : public DAGStage,
                          public ParameterizedStage,
                          public TriggerableStage {
 public:
  VideoDatSinkStage();
  ~VideoDatSinkStage() override = default;

  // DAGStage interface
  std::string version() const override { return "1.0"; }
  ORC_STAGE_INSTRUCTIONS_MD
  NodeTypeInfo get_node_type_info() const override;

  std::vector<ArtifactPtr> execute(
      const std::vector<ArtifactPtr>& inputs,
      const std::map<std::string, ParameterValue>& parameters,
      ObservationContext& observation_context) override;

  size_t required_input_count() const override { return 1; }
  size_t output_count() const override { return 0; }

  // ParameterizedStage interface
  std::vector<ParameterDescriptor> get_parameter_descriptors(
      VideoSystem project_format, SourceType source_type) const override;
  using ParameterizedStage::get_parameter_descriptors;
  std::map<std::string, ParameterValue> get_parameters() const override;
  bool set_parameters(
      const std::map<std::string, ParameterValue>& params) override;

  // TriggerableStage interface
  bool trigger(const std::vector<ArtifactPtr>& inputs,
               const std::map<std::string, ParameterValue>& parameters,
               IObservationContext& observation_context) override;

  std::string get_trigger_status() const override;

  void set_progress_callback(TriggerProgressCallback callback) override {
    progress_callback_ = callback;
  }

  bool is_trigger_in_progress() const override { return is_processing_.load(); }

  void cancel_trigger() override { cancel_requested_.store(true); }

  void set_deps_override(std::shared_ptr<IVideoDatSinkStageDeps> deps) {
    deps_override_ = std::move(deps);
  }

  // The folder the files go into: the report path without its extension, or
  // with "_files" appended when it has none.
  static std::string output_directory_for(const std::string& report_path);

  // Export options from a parameter map; throws when output_path is missing.
  static VideoDatExportOptions parse_options(
      const std::map<std::string, ParameterValue>& parameters);

 private:
  std::map<std::string, ParameterValue> parameters_;
  TriggerProgressCallback progress_callback_;
  std::atomic<bool> is_processing_{false};
  std::atomic<bool> cancel_requested_{false};
  std::shared_ptr<IVideoDatSinkStageDeps> deps_override_;
};

}  // namespace orc

#endif  // ORC_VIDEODAT_SINK_STAGE_H
