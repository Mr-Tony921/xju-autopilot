/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common//trajectory_stitcher/trajectory_stitcher.h"

#include "planning/common/planning_gflags/planning_gflags.h"
#include "common/math/vec2d.h"
#include "common/logger/logger.h"
#include "vehicle_signal.pb.h"
#include "common/math/math_utils.h"

namespace xju {
namespace planning {

std::vector<pnc::TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory (
    const pnc::VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time, const PublishableTrajectory* prev_trajectory) {
  if (!FLAGS_enable_trajectory_stitcher) {
    ADEBUG << "stitch is disabled by gflags.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  if (!prev_trajectory || prev_trajectory->empty()) {
    AINFO << "replan for nor previous trajectory.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }
  
  // if (vehicle_state.driving_mode() != pnc::DrivingMode::COMPLETE_AUTO_DRIVE) {
  //   AINFO << "replan for nor not in auto driving mode.";
  //   return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  // }
  
  const double veh_relative_time = 
      current_timestamp - prev_trajectory->header_time();
  size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_relative_time);
  if (time_matched_index == 0 &&
      veh_relative_time < prev_trajectory->StartPoint().relative_time()) {
    AINFO << "current time smaller than the previous trajectory's first time";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  auto time_matched_point = prev_trajectory->TrajectoryPointAt(
      static_cast<uint32_t>(time_matched_index));
  if (!time_matched_point.has_path_point()) {
    AINFO << "replan for previous trajectory missed path point.";
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  size_t position_matched_index = prev_trajectory->QueryNearestPoint(
      {vehicle_state.x(), vehicle_state.y()});

  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(position_matched_index));
  
  auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
  auto lat_diff = frenet_sd.second;

  AINFO << "Control lateral diff: " << lat_diff
        << ", longitudinal diff: " << lon_diff;
  
  if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold) {
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  if (std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
    return ComputeReinitStitchingTrajectory(planning_cycle_time, vehicle_state);
  }

  double forward_rel_time = veh_relative_time + planning_cycle_time;
  size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);
  ADEBUG << "Position matched index:\t" << position_matched_index;
  ADEBUG << "Time matched index:\t" << time_matched_index;

  auto matched_index = std::min(time_matched_index, position_matched_index);

  std::vector<pnc::TrajectoryPoint> stitching_trajectory(
      prev_trajectory->begin() +
          std::max(0, static_cast<int>(matched_index - FLAGS_trajectory_stitching_preserved_length)),
      prev_trajectory->begin() + forward_time_index + 1);
  ADEBUG << "stitching_trajectory size: " << stitching_trajectory.size();

  const double zero_s = stitching_trajectory.back().path_point().s();
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      AWARN << "replan for previous trajectory missed path point";
      return ComputeReinitStitchingTrajectory(planning_cycle_time,
                                              vehicle_state);
    }
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  return stitching_trajectory;
}

std::vector<pnc::TrajectoryPoint> 
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const double current_timestamp, const pnc::VehicleState& vehicle_state) {
  pnc::TrajectoryPoint reinit_point;
  reinit_point.mutable_path_point()->set_s(0.0);
  reinit_point.mutable_path_point()->set_x(vehicle_state.x());
  reinit_point.mutable_path_point()->set_y(vehicle_state.y());
  reinit_point.mutable_path_point()->set_z(vehicle_state.z());
  reinit_point.mutable_path_point()->set_theta(vehicle_state.heading());
  reinit_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  reinit_point.set_v(vehicle_state.linear_velocity());
  reinit_point.set_a(vehicle_state.linear_acceleration());
  reinit_point.set_relative_time(0.0);
  return std::vector<pnc::TrajectoryPoint>(1, reinit_point);
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const pnc::TrajectoryPoint& p) {
  pnc::Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  pnc::Vec2d n(std::cos(p.path_point().theta()), std::sin(p.path_point().theta()));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();
  frenet_sd.second = v.CrossProd(n);
  return frenet_sd;
}

void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* const prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  double cos_theta = std::cos(theta_diff);
  double sin_theta = -std::sin(theta_diff);

  double tx = -(cos_theta * x_diff - sin_theta * y_diff);
  double ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->begin(), prev_trajectory->end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](pnc::TrajectoryPoint& p) {
                  double x = p.path_point().x();
                  double y = p.path_point().y();
                  double theta = p.path_point().theta();

                  double x_new = cos_theta * x - sin_theta * y + tx;
                  double y_new = sin_theta * x + cos_theta * y + ty;
                  double theta_new =
                      pnc::NormalizeAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

} // namespace planning
} // namespace xju