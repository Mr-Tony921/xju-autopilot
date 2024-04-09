/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/emergency_stop/emergency_stop_scenario.h"
#include "planning/scenarios/emergency_stop/stage_approach.h"
#include "planning/scenarios/emergency_stop/stage_standby.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

std::unique_ptr<Stage> EmergencyStopScenario::CreateStage(
    const pnc::StageConfig& stage_config, 
    std::shared_ptr<PlanningInternal>& internal) {
  std::unique_ptr<Stage> stage_ptr = nullptr;
  pnc::StageType stage_type = stage_config.stage_type();
  ADEBUG << "EmergencyStopScenario::CreateStage, stage_type: " << stage_type;
  switch (stage_type) {
    case pnc::StageType::EMERGENCY_STOP_APPROACH: {
      stage_ptr = std::unique_ptr<Stage>(
          new EmergencyStopStageApproach(stage_config, internal));
      break;
    }
    case pnc::StageType::EMERGENCY_STOP_STANDBY: {
      stage_ptr = std::unique_ptr<Stage>(
          new EmergencyStopStageStandby(stage_config, internal));
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