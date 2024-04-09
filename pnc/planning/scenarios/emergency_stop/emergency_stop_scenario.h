/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/scenarios/scenario.h"

namespace xju {
namespace planning {

class EmergencyStopScenario : public Scenario {
 public:
  EmergencyStopScenario(const pnc::ScenarioConfig& config,
                     std::shared_ptr<PlanningInternal>& internal)
      : Scenario(config, internal) {}
  
  std::unique_ptr<Stage> CreateStage(
      const pnc::StageConfig& stage_config, 
      std::shared_ptr<PlanningInternal>& internal) override;
};

} // namespace planning
} // namespace xju
