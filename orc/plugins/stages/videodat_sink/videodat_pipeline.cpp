/*
 * File:        videodat_pipeline.cpp
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     The VideoDat decoder from frames to the files it writes
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_pipeline.h"

#include <spdlog/fmt/fmt.h>

#include <algorithm>
#include <utility>

#include "videodat_services.h"

namespace orc {

VideoDatPipelineResult run_videodat_packet_pipeline(
    const VideoDatByteStream& stream, const VideoDatPipelineOptions& options) {
  VideoDatPipelineResult result;
  std::vector<std::string>& log = result.log;

  if (options.save_byte_stream) {
    result.outputs.push_back(
        {kVideoDatByteStreamName, stream.data, std::nullopt});
  }

  std::vector<VideoDatPacket> packets = find_videodat_packets(stream);
  result.packets = packets.size();
  const size_t received_ok = static_cast<size_t>(
      std::count_if(packets.begin(), packets.end(),
                    [](const VideoDatPacket& p) { return p.ok; }));
  const size_t unstuffed = static_cast<size_t>(std::count_if(
      packets.begin(), packets.end(), [](const VideoDatPacket& p) {
        return p.recovery == VideoDatRecovery::kUnstuffed;
      }));
  const VideoDatRepairCounts repaired = repair_videodat_packets(packets);
  result.packets_ok = static_cast<size_t>(
      std::count_if(packets.begin(), packets.end(),
                    [](const VideoDatPacket& p) { return p.ok; }));
  log.push_back(fmt::format(
      "Packets: {} found, {} checksum ok, repaired: {} by soft decision, {} "
      "from copies",
      packets.size(), received_ok, repaired.soft, repaired.copies));
  if (unstuffed > 0) {
    log.push_back(fmt::format(
        "  {} packets valid after removing sync escape bytes", unstuffed));
  }

  VideoDatAssembly assembly =
      assemble_videodat_files(packets, options.assembly);
  for (const std::string& line : assembly.log) log.push_back("  " + line);

  log.push_back("");
  log.push_back("Files:");
  for (const VideoDatFileResult& file : assembly.files) {
    const std::string state =
        file.missing_blocks == 0
            ? std::string("complete")
            : fmt::format("incomplete ({} of {} blocks missing)",
                          file.missing_blocks, file.blocks);
    log.push_back(fmt::format(
        "  Stream {:02X}  {:<14} {:>8} bytes  {}{}{}", file.stream, file.name,
        file.size, state,
        file.transmissions > 1 ? fmt::format("  (sent {}x)", file.transmissions)
                               : std::string(),
        file.output.has_value() ? "" : "  [not written]"));
    if (file.output.has_value()) {
      const auto header =
          videodat_chvd_header(assembly.outputs[*file.output].data);
      if (header.has_value()) {
        log.push_back(fmt::format(
            "            purchased software (CHVDFILE): holds {}, {} bytes{}, "
            "encrypted",
            header->name, header->length,
            header->modified.has_value()
                ? fmt::format(" of {} {:02d}:{:02d}",
                              videodat_format_date(*header->modified),
                              header->modified->hour, header->modified->minute)
                : std::string()));
      }
    }
    ++result.files;
    if (file.missing_blocks == 0) ++result.complete_files;
  }
  log.push_back(fmt::format("{} files, {} complete", result.files,
                            result.complete_files));
  for (VideoDatOutputFile& output : assembly.outputs) {
    result.outputs.push_back(std::move(output));
  }

  if (options.analyse_services) {
    VideoDatServiceReport services = analyse_videodat_services(packets);
    log.push_back("");
    for (const std::string& line : services.lines) log.push_back(line);
    for (VideoDatOutputFile& output : services.outputs) {
      result.outputs.push_back(std::move(output));
    }
  }
  return result;
}

VideoDatPipelineResult run_videodat_pipeline(
    const IVideoDatFrameSource& source, const VideoDatPipelineOptions& options,
    const VideoDatProgressFn& progress,
    const std::function<bool()>& cancelled) {
  VideoDatDecodeResult decoded =
      decode_videodat_stream(source, options.decoder, progress, cancelled);
  if (decoded.cancelled) {
    VideoDatPipelineResult result;
    result.log = std::move(decoded.log);
    result.cancelled = true;
    return result;
  }
  VideoDatPipelineResult result =
      run_videodat_packet_pipeline(decoded.stream, options);
  result.log.insert(result.log.begin(), decoded.log.begin(), decoded.log.end());
  return result;
}

}  // namespace orc
