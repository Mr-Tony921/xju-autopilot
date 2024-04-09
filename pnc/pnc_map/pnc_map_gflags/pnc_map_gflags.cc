/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "pnc_map/pnc_map_gflags/pnc_map_gflags.h"

namespace xju {
namespace pnc_map {

// parameters for lane process
DEFINE_double(max_navigation_length, 260, "limit the navigation length");
DEFINE_bool(use_side_lane, true, "is use side lane");
DEFINE_bool(use_side_lane_in_lane_change, true, 
            "is use side lane in dealing with lane change ref line");
DEFINE_bool(use_line_pts, true, 
            "is use lane mark vec points generate ref line");
DEFINE_bool(use_center_line, true, 
            "is use center line generate ref line");
DEFINE_bool(enable_smooth_reference_line, false, 
            "is enable smooth reference line use osqp");
DEFINE_bool(enable_reference_line_pose_correction, true, 
            "is enable correct reference line between em & pnc locations");
DEFINE_bool(enable_lane_mark_tracking, true, 
            "is enable track same lane mark");
DEFINE_double(side_lane_offset_width, 2.7, "limit side lane offset");
DEFINE_double(lane_width_in_process_lane_change, 2.0, 
              "lane_width_in_process_lane_change");
} // namespace pnc_map
} // namespace xju
