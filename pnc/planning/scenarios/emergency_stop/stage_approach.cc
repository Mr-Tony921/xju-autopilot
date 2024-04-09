/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/emergency_stop/stage_approach.h"

#include "common/time/time.h"
#include "common/logger/logger.h"
#include "common/util/coordinate.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "planning/common/util/util.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace xju {
namespace planning {

constexpr double kDistanceBuffer = 2.0;

EmergencyStopStageApproach::EmergencyStopStageApproach(
    const pnc::StageConfig& config, std::shared_ptr<PlanningInternal>& internal)
    : Stage(config, internal) {
  if (config.has_lane_decider_config()) {
    lane_decider_ptr_.reset(
        new LaneDecider(config_.lane_decider_config(), internal_));
  } else {
    lane_decider_ptr_.reset(new LaneDecider(internal_));
  }
}

Stage::Status EmergencyStopStageApproach::Process(Frame* frame) {
  Stage::Process(frame);
  bool lane_decider_status = lane_decider_ptr_->Process(frame);
  if (!lane_decider_status) {
    return Stage::Status::ERROR;
  }

  std::shared_ptr<ReferenceLineInfo>  reference_line_info = nullptr;
  if (internal_->planning_status().lane_change_status().status() ==
      LaneChangeStatus::LANE_CHANGE) {
    auto reference_line_infos = frame->GetPrioritizedReferenceLineInfos();
    reference_line_info = reference_line_infos.front();
  } else {
    reference_line_info = frame->GetCurrentReferenceLineInfo();
  }

  const auto& reference_line = reference_line_info->reference_line();
  const double ego_speed =
      reference_line_info->vehicle_state().linear_velocity();
  const double ego_front_edge_s =
      reference_line_info->car_sl_boundary().end_s();
  const double stop_distance =
      scenario_config_->emergency_stop_config().stop_distance();

  bool stop_fence_exist = false;
  double stop_line_s;
  const auto& emergency_stop_status =
      internal_->planning_status().emergency_stop_status();
  auto localize_pose = pnc::VehicleStateProvider::localize_pose();

  if (emergency_stop_status.has_local_stop_fence_point()) {
    double stop_point_x = emergency_stop_status.local_stop_fence_point().x();
    double stop_point_y = emergency_stop_status.local_stop_fence_point().y();

    // transform stop fence point from local to vehicle coordinate 
    pnc::TransformLocalToVehicleCoord(localize_pose.x, localize_pose.y,
                                      localize_pose.heading, &stop_point_x,
                                      &stop_point_y);

    pnc::Vec2d veh_stop_fence_point(stop_point_x, stop_point_y);
    ADEBUG << "Stop fence point in vehicle coord.: (" << stop_point_x << ", "
           << stop_point_y << "), in local coord.: ("
           << emergency_stop_status.local_stop_fence_point().x() << ", "
           << emergency_stop_status.local_stop_fence_point().y() << ")";

    pnc::SLPoint stop_fence_sl;
    reference_line.XYToSL(veh_stop_fence_point, &stop_fence_sl);
    if (stop_fence_sl.s() > ego_front_edge_s) {
      stop_fence_exist = true;
      stop_line_s = stop_fence_sl.s();
    }
  }

  if (!stop_fence_exist) {
    const double deceleration =
        scenario_config_->emergency_stop_config().max_stop_deceleration();
    double travel_distance = ego_speed * ego_speed / 2 / deceleration;
    const double ego_accel =
        reference_line_info->vehicle_state().linear_acceleration();
    if (ego_accel > (-deceleration)) {
      double time_temp =
         (ego_accel - (-deceleration)) / (-FLAGS_longitudinal_jerk_lower_bound);
      double v_temp =
          ego_speed + ego_accel * time_temp +
          0.5 * FLAGS_longitudinal_jerk_lower_bound * std::pow(time_temp, 2);
      v_temp = std::max(v_temp, 0.0);
      travel_distance =
          ego_speed * time_temp + 0.5 * ego_accel * std::pow(time_temp, 2) +
          FLAGS_longitudinal_jerk_lower_bound * std::pow(time_temp, 3) / 6 +
          std::pow(v_temp, 2) / 2 / deceleration;
    }
    stop_line_s =
        ego_front_edge_s + travel_distance + kDistanceBuffer + stop_distance;
    ADEBUG << "stop_distance: " << stop_distance
           << ", max_stop_deceleration: " << deceleration
           << ", ego_speed: " << ego_speed
           << ", ego_accel: " << ego_accel
           << ", travel_distance: " << travel_distance
           << ", ego_front_edge_s: " << ego_front_edge_s
           << ", stop_line_s: " << stop_line_s;
    const auto& stop_fence_point = reference_line.GetReferencePoint(stop_line_s);
    double stop_point_x = stop_fence_point.x();
    double stop_point_y = stop_fence_point.y();
    // transform stop fence point from vehicle to localize coordinate 
    pnc::TransformVehicleToLocalCoord(localize_pose.x, localize_pose.y,
                                      localize_pose.heading, &stop_point_x,
                                      &stop_point_y);
    ADEBUG << "Stop fence point in vehicle coord.: (" << stop_fence_point.x()
           << ", " << stop_fence_point.y() << "), in local coord.: ("
           << stop_point_x << ", " << stop_point_y << ")";

    auto* emergency_stop_fence_point = internal_->mutable_planning_status()
                                           ->mutable_emergency_stop_status()
                                           ->mutable_local_stop_fence_point();
    emergency_stop_fence_point->set_x(stop_point_x);
    emergency_stop_fence_point->set_y(stop_point_y);
  }
  
  const std::string virtual_obstacle_id = "EMERGENCY_STOP";
  int ret = planning::util::BuildStopDecision(virtual_obstacle_id,
                                                 stop_line_s, stop_distance,
                                                 frame, reference_line_info);
  ADEBUG << "Build a stop fence for emergency_stop: id [" << virtual_obstacle_id
         << "], s [" << stop_line_s << "]";

  bool status = ProcessTaskOnReferenceLine(reference_line_info, frame);
  if (!status) {
    AERROR << "EmergencyStopStageApproach planning error";
  }

  double standstill_speed = pnc::VehicleConfigProvider::GetConfig()
                            .max_abs_speed_when_stopped();
  if (ego_speed <= standstill_speed) {
    ADEBUG << "ego_speed[" << ego_speed << "] <= standstill_speed["
           << standstill_speed << "], approach stage finished";
    return FinishStage();
  }

  return Stage::Status::PROCESSING;
}

Stage::Status EmergencyStopStageApproach::FinishStage() {
  next_stage_type_ = pnc::StageType::EMERGENCY_STOP_STANDBY;
  return Stage::Status::FINISHED;
}

} // namespace planning
} // namespace xju