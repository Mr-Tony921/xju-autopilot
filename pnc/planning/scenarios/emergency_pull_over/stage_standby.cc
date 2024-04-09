/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/emergency_pull_over/stage_standby.h"

#include "common/logger/logger.h"
#include "common/time/time.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "common/util/coordinate.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "planning/common/util/util.h"

namespace xju {
namespace planning {

EmergencyPullOverStageStandby::EmergencyPullOverStageStandby(
    const pnc::StageConfig& config, std::shared_ptr<PlanningInternal>& internal)
    : Stage(config, internal) {}

Stage::Status EmergencyPullOverStageStandby::Process(Frame* frame) {
  Stage::Process(frame);
  const auto& reference_line_info = frame->GetCurrentReferenceLineInfo();

  // TODO turn on vehicle emergency signal

  const auto& emergency_pull_over_status =
      internal_->planning_status().emergency_pull_over_status();
  auto localize_pose = pnc::VehicleStateProvider::localize_pose();
  auto car_config = pnc::VehicleConfigProvider::GetConfig();
  if (emergency_pull_over_status.has_position() &&
      emergency_pull_over_status.position().has_x() &&
      emergency_pull_over_status.position().has_y()) {
    const auto& reference_line = reference_line_info->reference_line();
    double pull_over_position_x = emergency_pull_over_status.position().x();
    double pull_over_position_y = emergency_pull_over_status.position().y();
    // transform pull over position from local to vehicle coordinate
    pnc::TransformLocalToVehicleCoord(
        localize_pose.x, localize_pose.y, localize_pose.heading,
        &pull_over_position_x, &pull_over_position_y);
    pnc::Vec2d pull_over_xy(pull_over_position_x, pull_over_position_y);
    ADEBUG << "Pull over position in vehicle coord.: (" << pull_over_position_x
           << ", " << pull_over_position_y << "), in local coord.: ("
           << emergency_pull_over_status.position().x() << ", "
           << emergency_pull_over_status.position().y() << ")";
    pnc::SLPoint pull_over_sl;
    reference_line.XYToSL(pull_over_xy, &pull_over_sl);
    const double stop_distance =
        scenario_config_->emergency_pull_over_config().stop_distance();
    double stop_line_s = pull_over_sl.s() + stop_distance +
                  car_config.front_edge_to_center() +
                  car_config.rear_axle_to_center();
    const double ego_front_edge_s =
        reference_line_info->car_sl_boundary().end_s();
    ADEBUG << "stop_line_s: " << stop_line_s
           << ", ego_front_edge_s: " << ego_front_edge_s;

    double distance = stop_line_s - ego_front_edge_s;
    if (distance <= 0.0) {
      stop_line_s = ego_front_edge_s + stop_distance;
    }
    ADEBUG << "stop_distance: " << stop_distance
           << ", pull_over_s: " << pull_over_sl.s()
           << ", stop_line_s: " << stop_line_s;

    const std::string virtual_obstacle_id = "EMERGENCY_PULL_OVER";
    int ret = planning::util::BuildStopDecision(virtual_obstacle_id,
                                                stop_line_s, stop_distance,
                                                frame, reference_line_info);
    ADEBUG << "Build a stop fence for emergency_pull_over: id ["
           << virtual_obstacle_id << "], s [" << stop_line_s << "]";
  }

  bool status = ProcessTaskOnReferenceLine(reference_line_info, frame);
  if (!status) {
    AERROR << "EmergencyPullOverStageApproach planning error";
  }

  return Stage::Status::PROCESSING;
}

Stage::Status EmergencyPullOverStageStandby::FinishStage() {
  return FinishScenario();
}

}  // namespace planning
}  // namespace xju