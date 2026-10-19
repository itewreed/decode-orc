/*
 * File:        videodat_pipeline.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     The VideoDat decoder from frames to the files it writes
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_PIPELINE_H
#define ORC_VIDEODAT_PIPELINE_H

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

#include "videodat_file_assembler.h"
#include "videodat_stream_decoder.h"

namespace orc {

struct VideoDatPipelineOptions {
  VideoDatDecoderOptions decoder;
  VideoDatAssemblyOptions assembly;
  // Report the clock, schedule, bit lists and encrypted services.
  bool analyse_services = true;
  // Also write the recovered byte stream (ten bytes per data line).
  bool save_byte_stream = false;
};

// Name of the byte stream among the outputs.
inline constexpr const char* kVideoDatByteStreamName = "videodat_bytes.bin";

struct VideoDatPipelineResult {
  // Files to write, relative to the output directory.
  std::vector<VideoDatOutputFile> outputs;
  // The report, line by line.
  std::vector<std::string> log;
  size_t packets = 0;
  size_t packets_ok = 0;
  size_t files = 0;
  size_t complete_files = 0;
  bool cancelled = false;
};

// Run the decoder over a recording: byte stream, packets, repair, file
// assembly and, optionally, the other services.
VideoDatPipelineResult run_videodat_pipeline(
    const IVideoDatFrameSource& source, const VideoDatPipelineOptions& options,
    const VideoDatProgressFn& progress = nullptr,
    const std::function<bool()>& cancelled = nullptr);

// The packet stages alone, for a byte stream already recovered.
VideoDatPipelineResult run_videodat_packet_pipeline(
    const VideoDatByteStream& stream, const VideoDatPipelineOptions& options);

}  // namespace orc

#endif  // ORC_VIDEODAT_PIPELINE_H
