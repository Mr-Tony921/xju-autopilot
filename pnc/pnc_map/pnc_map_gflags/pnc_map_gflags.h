/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "gflags/gflags.h"

namespace xju {
namespace pnc_map {

// parameters for lane process
DECLARE_double(max_navigation_length);
DECLARE_bool(use_side_lane);
DECLARE_bool(use_side_lane_in_lane_change);
DECLARE_bool(use_line_pts);
DECLARE_bool(use_center_line);
DECLARE_bool(enable_smooth_reference_line);
DECLARE_bool(enable_reference_line_pose_correction);
DECLARE_bool(enable_lane_mark_tracking);
DECLARE_double(side_lane_offset_width);
DECLARE_double(lane_width_in_process_lane_change);
} // namespace planning
} // namespace xju
