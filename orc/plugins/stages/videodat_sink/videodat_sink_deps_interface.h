/*
 * File:        videodat_sink_deps_interface.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Interface for VideoDatSinkStage dependencies
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_SINK_DEPS_INTERFACE_H
#define ORC_VIDEODAT_SINK_DEPS_INTERFACE_H

#include <orc/stage/triggerable_stage.h>
#include <orc/stage/video_frame_representation.h>

#include <atomic>
#include <cstddef>
#include <string>

#include "videodat_pipeline.h"

namespace orc {

struct VideoDatExportOptions {
  // The report; the decoded files go into output_directory.
  std::string report_path;
  std::string output_directory;
  VideoDatPipelineOptions pipeline;
};

struct VideoDatExportResult {
  bool success = false;
  std::string message;
  size_t packets_ok = 0;
  size_t files = 0;
  size_t complete_files = 0;
};

class IVideoDatSinkStageDeps {
 public:
  virtual ~IVideoDatSinkStageDeps() = default;

  virtual void init(TriggerProgressCallback progress_callback,
                    std::atomic<bool>* cancel_requested) = 0;

  virtual VideoDatExportResult export_videodat(
      const VideoFrameRepresentation* representation,
      const VideoDatExportOptions& options) = 0;
};

}  // namespace orc

#endif  // ORC_VIDEODAT_SINK_DEPS_INTERFACE_H
