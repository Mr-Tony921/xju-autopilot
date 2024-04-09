/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/common/trajectory/discretized_trajectory.h"

namespace xju {
namespace planning {

class ConstraintChecker {
 public:
  ConstraintChecker() = delete;
  static bool ValidTrajectroy(const DiscretizedTrajectory& trajectory);
};

} // namespace planning
} // namespace xju
