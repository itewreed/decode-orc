/*
 * File:        videodat_stream_decoder.h
 * Module:      orc-stage-plugin-videodat_sink
 * Purpose:     Recovers the VideoDat byte stream from the VBI of PAL frames
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 decode-orc contributors
 */

#ifndef ORC_VIDEODAT_STREAM_DECODER_H
#define ORC_VIDEODAT_STREAM_DECODER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "videodat_line_slicer.h"
#include "videodat_packet.h"

namespace orc {

// The lines of a recording, field by field.
class IVideoDatFrameSource {
 public:
  virtual ~IVideoDatFrameSource() = default;

  virtual uint64_t frame_count() const = 0;

  // Copy line `line` (0-based) of field `field` (0 the first, 1 the second)
  // of frame `frame` into `out`.  Returns false when the line is not there.
  virtual bool read_line(uint64_t frame, int field, size_t line,
                         std::vector<int16_t>& out) const = 0;
};

struct VideoDatDecoderOptions {
  // Use the end of line sync, where the input carries it, as the timing
  // reference of each line: a line whose start edge is damaged is then still
  // read.
  bool use_sync = true;
  // Train an equaliser on the recording's own error-free lines.
  bool use_equaliser = true;
  // On input without sync (a VBI capture), follow data lines that move within
  // the field from frame to frame.
  bool track_lines = true;
  // The line search reads at most this many seconds from the start.
  double scan_seconds = 30.0;

  VideoDatSlicerConfig slicer;
};

struct VideoDatDecodeResult {
  VideoDatByteStream stream;
  // Data lines per field, 0-based field line indexes.
  std::array<std::vector<size_t>, 2> lines;
  std::optional<double> sync_offset;
  std::optional<VideoDatEqualiser> equaliser;
  std::vector<std::string> log;
  bool cancelled = false;
};

using VideoDatProgressFn =
    std::function<void(uint64_t done, uint64_t total, const std::string&)>;

// Field lines searched for data, from the top of each field.
inline constexpr size_t kVideoDatSearchLines = 40;

// Find the data lines, learn the timing and the equaliser, and slice every
// data line of the recording into the byte stream.
//
// The data lines are found by their framing (start and stop bits of all ten
// bytes, at most two wrong) rather than by the packet sync word, which lies
// whole in a line only now and then.  Lines are counted per field parity in
// blocks of a second until both parities have enough hits and the choice has
// not changed for five blocks.
VideoDatDecodeResult decode_videodat_stream(
    const IVideoDatFrameSource& source, const VideoDatDecoderOptions& options,
    const VideoDatProgressFn& progress = nullptr,
    const std::function<bool()>& cancelled = nullptr);

}  // namespace orc

#endif  // ORC_VIDEODAT_STREAM_DECODER_H
