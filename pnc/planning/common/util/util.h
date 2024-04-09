/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/common/frame/frame.h"
#include "planning/common/reference_line_info/reference_line_info.h"
#include "planning/common/planning_internal/planning_internal.h"

namespace xju {
namespace planning {
namespace util {

int BuildStopDecision(
    const std::string& stop_wall_id, const double stop_line_s,
    const double stop_distance, Frame* const frame,
    std::shared_ptr<ReferenceLineInfo> const reference_line_info);

}  // namespace util
}  // namespace planning
}  // namespace xju
