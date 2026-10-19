/*
 * File:        videodat_sink_deps.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     VideoDatSinkStage dependency implementation
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_SINK_DEPS_H
#define ORC_VIDEODAT_SINK_DEPS_H

#include <atomic>
#include <string>
#include <vector>

#include "videodat_sink_deps_interface.h"

namespace orc {

// Reads the frames of the input representation and writes the decoder's
// output to disk.
class VideoDatSinkStageDeps : public IVideoDatSinkStageDeps {
 public:
  void init(TriggerProgressCallback progress_callback,
            std::atomic<bool>* cancel_requested) override;

  VideoDatExportResult export_videodat(
      const VideoFrameRepresentation* representation,
      const VideoDatExportOptions& options) override;

 private:
  TriggerProgressCallback progress_callback_;
  std::atomic<bool>* cancel_requested_ = nullptr;
};

// The lines of a representation's frames, as the decoder reads them.  The
// second field of a 625-line frame starts at frame line 313.
class VideoDatRepresentationSource : public IVideoDatFrameSource {
 public:
  explicit VideoDatRepresentationSource(
      const VideoFrameRepresentation& representation);

  uint64_t frame_count() const override;
  bool read_line(uint64_t frame, int field, size_t line,
                 std::vector<int16_t>& out) const override;

 private:
  const VideoFrameRepresentation& representation_;
  FrameID first_ = 0;
  uint64_t count_ = 0;
};

}  // namespace orc

#endif  // ORC_VIDEODAT_SINK_DEPS_H
