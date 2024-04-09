/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/road_change/road_change_scenario.h"

#include "planning/scenarios/road_change/road_change_stage.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

std::unique_ptr<Stage> RoadChangeScenario::CreateStage(
    const pnc::StageConfig& stage_config, 
    std::shared_ptr<PlanningInternal>& internal) {
  std::unique_ptr<Stage> stage_ptr = nullptr;
  pnc::StageType stage_type = stage_config.stage_type();
  switch (stage_type) {
    case pnc::StageType::ROAD_CHANGE_DEFAULT_STAGE: {
      stage_ptr = std::unique_ptr<Stage>(new RoadChangeStage(stage_config, internal));
      break;
    }
    default: {
      stage_ptr = nullptr;
      break;
    }
  }
  return stage_ptr;
}

} // namespace planning
} // namespace xju