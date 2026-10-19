/*
 * File:        vbi_line_mapping.cpp
 * Module:      orc-stage-plugin-vbi_source
 * Purpose:     Maps stored VBI line records onto CVBS frame lines
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 * SPDX-FileCopyrightText: 2026 Simon Inns
 */

#include "vbi_line_mapping.h"

namespace orc {

namespace {

// Stored fields per frame.  Every format in scope stores two.
constexpr uint32_t kStoredFieldsPerFrame = 2;

std::string tv_system_name(VBITVSystem tv_system) {
  switch (tv_system) {
    case VBITVSystem::kPAL:
      return "PAL";
    case VBITVSystem::kNTSC:
      return "NTSC";
    case VBITVSystem::kPALM:
      return "PAL_M";
  }
  return "unknown";
}

}  // namespace

const std::vector<uint32_t>& VBITeletextLineMap::for_tv_field(
    uint32_t tv_field) const {
  if (tv_field == 1) {
    return field1;
  }
  if (tv_field == 2) {
    return field2;
  }
  static const std::vector<uint32_t> kNoLines;
  return kNoLines;
}

bool make_vbi_teletext_line_map(VBITVSystem tv_system,
                                VBITeletextSystem tt_system,
                                VBITeletextLineMap& out_line_map,
                                std::string& error_message) {
  out_line_map = VBITeletextLineMap{};

  // VideoDat shares the list: its three lines a field sit inside it, and a card
  // stores the whole of it (see standard_teletext_lines_per_field).
  if (tv_system == VBITVSystem::kPAL &&
      (tt_system == VBITeletextSystem::kWST ||
       tt_system == VBITeletextSystem::kVideoDat)) {
    // ETSI EN 300 706 Section 4.1: World System Teletext occupies broadcast
    // frame lines 7-22 in field 1 and 320-335 in field 2 of a 625-line
    // system.  Stored frame lines are those numbers less one, the CVBS spec
    // placing stored frame line 0 at broadcast frame line 1.  Written out
    // entry by entry rather than derived, so that the table is the thing
    // being reviewed against the standard.
    out_line_map.field1 = {6,  7,  8,  9,  10, 11, 12, 13,
                           14, 15, 16, 17, 18, 19, 20, 21};
    out_line_map.field2 = {319, 320, 321, 322, 323, 324, 325, 326,
                           327, 328, 329, 330, 331, 332, 333, 334};
    return true;
  }

  if ((tv_system == VBITVSystem::kNTSC || tv_system == VBITVSystem::kPALM) &&
      tt_system != VBITeletextSystem::kVideoDat) {
    // ITU-R BT.653 §2: on 525-line systems both teletext services occupy
    // broadcast frame lines 10-21 in field 1 and 273-284 in field 2 — field
    // line 10 to 21 of each, field 2 starting at broadcast frame line 264.
    // Stored frame lines are those numbers less one.  Written out entry by
    // entry rather than derived, so that the table is the thing being reviewed
    // against the standard.
    out_line_map.field1 = {9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    out_line_map.field2 = {272, 273, 274, 275, 276, 277,
                           278, 279, 280, 281, 282, 283};
    return true;
  }

  error_message = "teletext.system " + to_string(tt_system) +
                  " has no defined line list on " + tv_system_name(tv_system) +
                  "; WST is defined on PAL and on 525-line systems, NABTS "
                  "on NTSC/PAL_M, and VideoDat on PAL.";
  return false;
}

uint32_t vbi_tv_field_for_stored_field(const VBISourceFormat& format,
                                       uint32_t stored_field_index) {
  const uint32_t first_field = (format.first_field == 2) ? 2u : 1u;
  const bool is_second_stored_field = (stored_field_index % 2u) == 1u;
  if (!is_second_stored_field) {
    return first_field;
  }
  return (first_field == 1) ? 2u : 1u;
}

bool map_vbi_record_to_frame_line(const VBISourceFormat& format,
                                  const VBITeletextLineMap& line_map,
                                  uint32_t stored_field_index,
                                  uint32_t record_index,
                                  uint32_t& out_frame_line,
                                  std::string& error_message) {
  out_frame_line = 0;

  if (stored_field_index >= kStoredFieldsPerFrame) {
    error_message = "Stored field index " + std::to_string(stored_field_index) +
                    " is out of range; a frame stores two sequential fields.";
    return false;
  }

  if (record_index < format.field_range.start ||
      record_index > format.field_range.end) {
    error_message = "Record " + std::to_string(record_index) +
                    " is outside container.field_range (" +
                    std::to_string(format.field_range.start) + ".." +
                    std::to_string(format.field_range.end) +
                    "), so it carries no data service.";
    return false;
  }

  const uint32_t tv_field =
      vbi_tv_field_for_stored_field(format, stored_field_index);
  const std::vector<uint32_t>& lines = line_map.for_tv_field(tv_field);

  // Useful records are stored contiguously, so a source carrying fewer lines
  // than the standard starts at the head of the list.
  const uint32_t table_index = record_index - format.field_range.start;
  if (table_index >= lines.size()) {
    error_message = "Record " + std::to_string(record_index) +
                    " maps past the end of the " + to_string(format.tt_system) +
                    " line list for television field " +
                    std::to_string(tv_field) + ", which defines " +
                    std::to_string(lines.size()) +
                    " lines; the excess records have no frame line to map to.";
    return false;
  }

  out_frame_line = lines[table_index];
  return true;
}

}  // namespace orc
