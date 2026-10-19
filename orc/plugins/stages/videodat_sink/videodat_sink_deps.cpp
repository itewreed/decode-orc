/*
 * File:        videodat_sink_deps.cpp
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     VideoDatSinkStage dependency implementation
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#include "videodat_sink_deps.h"

#include <orc/support/logging.h>

#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <system_error>
#include <utility>

namespace orc {

namespace {

// First frame line of the second field of a 625-line frame.
constexpr size_t kSecondFieldFirstLine = 313;

bool write_file(const std::filesystem::path& path,
                const std::vector<uint8_t>& data, std::string& error) {
  std::error_code ec;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), ec);
  }
  if (ec) {
    error = "Cannot create directory " + path.parent_path().string() + ": " +
            ec.message();
    return false;
  }
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) {
    error = "Cannot open " + path.string() + " for writing";
    return false;
  }
  out.write(reinterpret_cast<const char*>(data.data()),
            static_cast<std::streamsize>(data.size()));
  if (!out) {
    error = "Cannot write " + path.string();
    return false;
  }
  return true;
}

// Give a file the transmitted modification time, a local time.  A failure
// leaves the file as written.
void set_modified(const std::filesystem::path& path,
                  const VideoDatDateTime& modified) {
  std::tm local{};
  local.tm_year = modified.year - 1900;
  local.tm_mon = modified.month - 1;
  local.tm_mday = modified.day;
  local.tm_hour = modified.hour;
  local.tm_min = modified.minute;
  local.tm_sec = modified.second;
  local.tm_isdst = -1;
  const std::time_t when = std::mktime(&local);
  if (when == static_cast<std::time_t>(-1)) return;
  // The file clock's epoch is unspecified before C++20: go through the
  // difference from now on both clocks.
  const auto system_now = std::chrono::system_clock::now();
  const auto file_now = std::filesystem::file_time_type::clock::now();
  const auto target = std::chrono::system_clock::from_time_t(when);
  std::error_code ec;
  std::filesystem::last_write_time(
      path,
      file_now +
          std::chrono::duration_cast<std::filesystem::file_time_type::duration>(
              target - system_now),
      ec);
}

std::filesystem::path output_path(const std::string& directory,
                                  const std::string& relative) {
  return std::filesystem::u8path(directory) / std::filesystem::u8path(relative);
}

}  // namespace

VideoDatRepresentationSource::VideoDatRepresentationSource(
    const VideoFrameRepresentation& representation)
    : representation_(representation) {
  const FrameIDRange range = representation.frame_range();
  first_ = range.first;
  count_ = range.count();
}

uint64_t VideoDatRepresentationSource::frame_count() const { return count_; }

bool VideoDatRepresentationSource::read_line(uint64_t frame, int field,
                                             size_t line,
                                             std::vector<int16_t>& out) const {
  const FrameID id = first_ + frame;
  const size_t flat_line = line + (field == 0 ? 0 : kSecondFieldFirstLine);
  if (representation_.has_separate_channels()) {
    // The data rides in the luma of a YC source.
    const auto params = representation_.get_video_parameters();
    const int16_t* luma = representation_.get_line_luma(id, flat_line);
    if (luma == nullptr || !params.has_value()) return false;
    out.assign(luma, luma + params->frame_width_nominal);
  } else {
    out = representation_.get_line_samples(id, flat_line);
  }
  return !out.empty();
}

void VideoDatSinkStageDeps::init(TriggerProgressCallback progress_callback,
                                 std::atomic<bool>* cancel_requested) {
  progress_callback_ = std::move(progress_callback);
  cancel_requested_ = cancel_requested;
}

VideoDatExportResult VideoDatSinkStageDeps::export_videodat(
    const VideoFrameRepresentation* representation,
    const VideoDatExportOptions& options) {
  VideoDatExportResult result;
  if (representation == nullptr) {
    result.message = "Input representation is null";
    return result;
  }
  const FrameIDRange range = representation->frame_range();
  if (range.count() == 0) {
    result.message = "Input has no frames";
    return result;
  }
  const auto descriptor = representation->get_frame_descriptor(range.first);
  if (!descriptor.has_value() || descriptor->system != VideoSystem::PAL) {
    result.message = "VideoDat is a PAL service; the input is not PAL";
    return result;
  }

  const VideoDatRepresentationSource source(*representation);
  const auto progress = [this](uint64_t done, uint64_t total,
                               const std::string& message) {
    if (progress_callback_) {
      progress_callback_(static_cast<size_t>(done), static_cast<size_t>(total),
                         message);
    }
  };
  const auto cancelled = [this]() {
    return cancel_requested_ != nullptr && cancel_requested_->load();
  };
  const VideoDatPipelineResult pipeline =
      run_videodat_pipeline(source, options.pipeline, progress, cancelled);
  if (pipeline.cancelled) {
    result.message = "Cancelled";
    return result;
  }

  for (const std::string& line : pipeline.log) {
    ORC_LOG_INFO("VideoDat: {}", line);
  }

  std::string error;
  for (const VideoDatOutputFile& output : pipeline.outputs) {
    const auto path = output_path(options.output_directory, output.path);
    if (!write_file(path, output.data, error)) {
      result.message = error;
      return result;
    }
    if (output.modified.has_value()) set_modified(path, *output.modified);
  }
  std::vector<uint8_t> report;
  for (const std::string& line : pipeline.log) {
    report.insert(report.end(), line.begin(), line.end());
    report.push_back('\n');
  }
  if (!write_file(std::filesystem::u8path(options.report_path), report,
                  error)) {
    result.message = error;
    return result;
  }

  result.success = true;
  result.packets_ok = pipeline.packets_ok;
  result.files = pipeline.files;
  result.complete_files = pipeline.complete_files;
  return result;
}

}  // namespace orc
