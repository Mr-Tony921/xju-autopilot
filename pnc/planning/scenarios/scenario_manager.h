/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "planning/scenarios/scenario.h"
#include "planning/common/frame/frame.h"
#include "planning/common/planning_internal/planning_internal.h"
#include "scenario_config.pb.h"


namespace xju {
namespace planning {

class ScenarioManager {
 public:
  ScenarioManager() = default;
  ~ScenarioManager() = default;

  bool Init(const std::shared_ptr<PlanningInternal>& interval);
  void Update(const Frame& frame);
  Scenario* current_scenario() {
    return current_scenario_.get();
  }
  
 private:
  bool RegisterScenarios();
  std::unique_ptr<Scenario> CreateScenario(const pnc::ScenarioType& scenario_type);
  void Observe(const Frame& frame);
  void ScenarioDispatch(const Frame& frame);
  void UpdatePlanningInternal(const Frame& frame,
                              const pnc::ScenarioType& scenario_type);

 private:
  pnc::ScenarioType default_scenario_type_;
  std::unique_ptr<Scenario> current_scenario_;
  std::unordered_map<pnc::ScenarioType, pnc::ScenarioConfig, std::hash<int>> scenario_config_map_;
  std::shared_ptr<PlanningInternal> internal_;
};

} // namespace planning
} // namespace xju
