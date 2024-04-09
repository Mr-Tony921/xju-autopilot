/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/emergency_pull_over/stage_slow_down.h"

#include "common/time/time.h"
#include "common/logger/logger.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace xju {
namespace planning {

constexpr double kSpeedTolarence = 1.0;

EmergencyPullOverStageSlowDown::EmergencyPullOverStageSlowDown(
    const pnc::StageConfig& config, std::shared_ptr<PlanningInternal>& internal)
    : Stage(config, internal) {}

Stage::Status EmergencyPullOverStageSlowDown::Process(Frame* frame) {
  Stage::Process(frame);
  const auto& reference_line_info = frame->GetCurrentReferenceLineInfo();

  const double target_slow_down_speed =
      scenario_config_->emergency_pull_over_config().target_slow_down_speed();
  reference_line_info->set_cruise_speed(target_slow_down_speed);

  bool status = ProcessTaskOnReferenceLine(reference_line_info, frame);
  if (!status) {
    AERROR << "EmergencyPullOverStageSlowDown planning error";
  }

  const double ego_speed =
      reference_line_info->vehicle_state().linear_velocity();
  if (ego_speed - target_slow_down_speed <= kSpeedTolarence) {
    ADEBUG << "ego_speed[" << ego_speed << "] <= target_slow_down_speed["
           << target_slow_down_speed << "] + kSpeedTolarence["
           << kSpeedTolarence << "], slow down stage finished";
    return FinishStage();
  }

  return Stage::Status::PROCESSING;
}

Stage::Status EmergencyPullOverStageSlowDown::FinishStage() {
  auto* pull_over_status = internal_->mutable_planning_status()
                               ->mutable_emergency_pull_over_status();
  pull_over_status->set_plan_pull_over_path(true);

  next_stage_type_ = pnc::StageType::EMERGENCY_PULL_OVER_APPROACH;
  return Stage::Status::FINISHED;
}


} // namespace planning
} // namespace xju