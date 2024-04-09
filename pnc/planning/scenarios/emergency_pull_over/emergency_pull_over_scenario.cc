/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/emergency_pull_over/emergency_pull_over_scenario.h"
#include "planning/scenarios/emergency_pull_over/stage_lane_change.h"
#include "planning/scenarios/emergency_pull_over/stage_slow_down.h"
#include "planning/scenarios/emergency_pull_over/stage_approach.h"
#include "planning/scenarios/emergency_pull_over/stage_standby.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

std::unique_ptr<Stage> EmergencyPullOverScenario::CreateStage(
    const pnc::StageConfig& stage_config, 
    std::shared_ptr<PlanningInternal>& internal) {
  std::unique_ptr<Stage> stage_ptr = nullptr;
  pnc::StageType stage_type = stage_config.stage_type();
  ADEBUG << "EmergencyPullOverScenario::CreateStage, stage_type: " << stage_type;
  switch (stage_type) {
    case pnc::StageType::EMERGENCY_PULL_OVER_LANE_CHANGE: {
      stage_ptr = std::unique_ptr<Stage>(
          new EmergencyPullOverStageLaneChange(stage_config, internal));
      break;
    }
    case pnc::StageType::EMERGENCY_PULL_OVER_SLOW_DOWN: {
      stage_ptr = std::unique_ptr<Stage>(
          new EmergencyPullOverStageSlowDown(stage_config, internal));
      break;
    }
    case pnc::StageType::EMERGENCY_PULL_OVER_APPROACH: {
      stage_ptr = std::unique_ptr<Stage>(
          new EmergencyPullOverStageApproach(stage_config, internal));
      break;
    }
    case pnc::StageType::EMERGENCY_PULL_OVER_STANDBY: {
      stage_ptr = std::unique_ptr<Stage>(
          new EmergencyPullOverStageStandby(stage_config, internal));
      break;
    }
    default: {
      stage_ptr = nullptr;
      break;
    }
  }
  if (stage_ptr) {
    stage_ptr->SetScenarioConfig(&config_);
  }
  return stage_ptr;
}

} // namespace planning
} // namespace xju