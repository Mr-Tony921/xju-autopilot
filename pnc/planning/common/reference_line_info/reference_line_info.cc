/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/reference_line_info/reference_line_info.h"

#include <cmath>

#include "common/logger/logger.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common/util/vehicle_helper.h"
#include "common/math/vec2d.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace xju {
namespace planning {

namespace {
  using ReferenceLine = pnc_map::ReferenceLine;
  using pnc::Box2d;
  using pnc::VehicleConfigProvider;

  constexpr double kOutOfReferenceLineL = 10.0;  // in meters
}  // namespace


ReferenceLineInfo::ReferenceLineInfo(const pnc::VehicleState& vehicle_state,
                                     const pnc::TrajectoryPoint& planning_start_point,
                                     const pnc_map::ReferenceLine& reference_line)
    : vehicle_state_(vehicle_state),
      planning_start_point_(planning_start_point),
      reference_line_(reference_line) {}

bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
  const auto& path_point = planning_start_point_.path_point();
  Box2d car_box = pnc::VehicleHelper::CarBox(path_point);
  
  if (!reference_line_.GetSLBoundary(car_box, car_sl_boundary_)) {
    AERROR << "Failed to get Car boundary from box: " << car_box.DebugString();
    return false;
  }

  adc_sl_boundary_ = car_sl_boundary_;

  if (adc_sl_boundary_.end_s() < 0 ||
      adc_sl_boundary_.start_s() > reference_line_.length()) {
    AWARN << "Car SL " << adc_sl_boundary_.ShortDebugString()
          << " is not on reference line:[0, " << reference_line_.length()
          << "]";
  }

  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
    AERROR << "Vehicle is too far away from reference line.";
    return false;
  }
  
  is_change_lane_ = !(reference_line_.IsOnLane(
      pnc::Vec2d(vehicle_state_.x(), vehicle_state_.y())));
  is_adc_on_reference_line_ = reference_line_.IsOnLane(adc_sl_boundary_);
  if (!AddObstacles(obstacles)) {
    AERROR << "Failed to add obstacles to reference line";
    return false;
  }
  set_cruise_speed(FLAGS_default_cruise_speed);
  return true;
}

Obstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    AERROR << "The provided obstacle is empty";
    return nullptr;
  }
  auto* mutable_obstacle = path_decision_.AddObstacle(*obstacle);
  if (!mutable_obstacle) {
    AERROR << "failed to add obstacle " << obstacle->id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->bounding_box(),
                                     perception_sl)) {
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->id();
    return mutable_obstacle;
  }
  mutable_obstacle->set_sl_boundary(perception_sl);

  if (IsIrrelevantObstacle(*mutable_obstacle)) {
    ObjectDecisionType ignore;
    ignore.mutable_ignore();
    path_decision_.SetLateralDecision(obstacle->id(), ignore);
    path_decision_.SetLongitudinalDecision(obstacle->id(), ignore);
    ADEBUG << "Irrelevant Obstacle id: " << obstacle->id();
  }
  return mutable_obstacle;
}

bool ReferenceLineInfo::AddObstacles(
    const std::vector<const Obstacle*>& obstacles) {
  //TODO: add multi thread to add obstacles
  for (const auto* obstacle : obstacles) {
    if (!AddObstacle(obstacle)) {
      AERROR << "Failed to add obstacle " << obstacle->id();
      return false;
    }
  }
  return true;
}

bool ReferenceLineInfo::IsIrrelevantObstacle(const Obstacle& obstacle) {
  if (obstacle.is_caution()) {
    return false;
  }
  // if adc is on the road, and obstacle behind adc, ignore
  const auto& obstacle_boundary = obstacle.sl_boundary();
  if (obstacle_boundary.end_s() > reference_line_.length()) {
    ADEBUG << "reference line length: " << reference_line_.length() 
           << " , obstacle_boundary end_s: " << obstacle_boundary.end_s();
    return true;
  }
  if (obstacle_boundary.start_l() > kOutOfReferenceLineL ||
      obstacle_boundary.end_l() < -kOutOfReferenceLineL) {
      ADEBUG << "obstacle_boundary.start_l: " << obstacle_boundary.start_l() 
             << " , obstacle_boundary.end_l: " << obstacle_boundary.end_l();
    return true;
  }
  return false;
}

void ReferenceLineInfo::SetBlockingObstacle(
    const std::string& blocking_obstacle_id) {
  blocking_obstacle_ = path_decision_.Find(blocking_obstacle_id);
}

const pnc::VehicleState& ReferenceLineInfo::vehicle_state() const { 
  return vehicle_state_; 
}

const pnc::TrajectoryPoint& ReferenceLineInfo::planning_start_point() const { 
  return planning_start_point_; 
}

const PathData& ReferenceLineInfo::path_data() const { 
  return path_data_;
}

PathData* ReferenceLineInfo::mutable_path_data() { 
  return &path_data_; 
}

const SpeedData& ReferenceLineInfo::speed_data() const { 
  return speed_data_; 
}

SpeedData* ReferenceLineInfo::mutable_speed_data() { 
  return &speed_data_; 
}

const std::vector<PathBoundary>& ReferenceLineInfo::path_boundary() const {
  return path_boundary_;
}

std::vector<PathBoundary>* ReferenceLineInfo::mutable_path_boundary() {
  return &path_boundary_;
}

const StGraphData& ReferenceLineInfo::st_graph_data() const { 
  return st_graph_data_; 
}

StGraphData* ReferenceLineInfo::mutable_st_graph_data() { 
  return &st_graph_data_; 
}

PathDecision* ReferenceLineInfo::path_decision() {
  return &path_decision_;
}

const PathDecision& ReferenceLineInfo::path_decision() const {
  return path_decision_;
}

const double ReferenceLineInfo::cruise_speed() const {
  return cruise_speed_ > 0.0 ? cruise_speed_ : FLAGS_default_cruise_speed;
}

void ReferenceLineInfo::set_cruise_speed(const double speed) {
  cruise_speed_ = speed; 
}

const ReferenceLine& ReferenceLineInfo::reference_line() const {
  return reference_line_;
}

const std::string ReferenceLineInfo::path_id() const {
  return reference_line_.id();
}

bool ReferenceLineInfo::CombinePathAndSpeedProfile(
    DiscretizedTrajectory* ptr_discretized_trajectory) {
  ACHECK(ptr_discretized_trajectory != nullptr);
  
  const double relative_time = planning_start_point_.relative_time();
  const double start_s = planning_start_point_.path_point().s();
  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;

  if (path_data_.planned_path().empty()) {
    AERROR << "path data is empty.";
    return false;
  }

  if (speed_data_.empty()) {
    AERROR << "speed data is empty.";
    return false;
  }

  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime(); ) {
    pnc::SpeedPoint speed_point;
    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.TotalLength()) {
      break;
    }
    pnc::PathPoint path_point = path_data_.GetPathPointByS(speed_point.s());
    path_point.set_s(speed_point.s() + start_s);
    pnc::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);

    cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion
                                                  : kSparseTimeResolution);
  }
  return true;
}

void ReferenceLineInfo::set_trajectory(
    const DiscretizedTrajectory& trajectory) {
  trajectory_ = trajectory;
}

const DiscretizedTrajectory& ReferenceLineInfo::trajectory() const {
  return trajectory_;
}

} // namespace planning
} // namespace xju
