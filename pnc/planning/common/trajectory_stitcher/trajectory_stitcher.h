/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "planning/common/trajectory/publishable_trajectory.h"
#include "vehicle_state.pb.h"

namespace xju {
namespace planning {

class TrajectoryStitcher {

 public:
  static std::vector<pnc::TrajectoryPoint> ComputeStitchingTrajectory (
      const pnc::VehicleState& vehicle_state, const double current_timestamp,
      const double planning_cycle_time, const PublishableTrajectory* prev_trajectory);

  static void TransformLastPublishedTrajectory(
      const double x_diff, const double y_diff, const double theta_diff,
      PublishableTrajectory* const prev_trajectory);

 private:
  static std::vector<pnc::TrajectoryPoint> ComputeReinitStitchingTrajectory(
      const double current_timestamp, const pnc::VehicleState& vehicle_state);

  static std::pair<double, double> ComputePositionProjection(
    const double x, const double y, const pnc::TrajectoryPoint& p);

  TrajectoryStitcher() = delete;
  TrajectoryStitcher(const TrajectoryStitcher&) = delete;
  TrajectoryStitcher &operator=(const TrajectoryStitcher&) = delete;
};

} // namespace planning
} // namespace xju
