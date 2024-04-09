/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/scenario_manager.h"

#include "planning/scenarios/lane_follow/lane_follow_scenario.h"
#include "planning/scenarios/road_change/road_change_scenario.h"
#include "planning/scenarios/emergency_stop/emergency_stop_scenario.h"
#include "planning/scenarios/emergency_pull_over/emergency_pull_over_scenario.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "planning_scenario_config.pb.h"
#include "common/logger/logger.h"
#include "common/vehicle_state/vehicle_state_provider.h"

namespace xju {
namespace planning {

bool ScenarioManager::Init(const std::shared_ptr<PlanningInternal>& internal) {
  internal_ = internal;
  RegisterScenarios();
  default_scenario_type_ = pnc::ScenarioType::LANE_FOLLOW;
  current_scenario_ = CreateScenario(default_scenario_type_);
  return true;
}

void ScenarioManager::Update(const Frame& frame) {
  Observe(frame);

  ScenarioDispatch(frame);
}

void ScenarioManager::Observe(const Frame& frame) {

}

void ScenarioManager::ScenarioDispatch(const Frame& frame) {
  // default: LANE_FOLLOW
  pnc::ScenarioType scenario_type = current_scenario_->scenario_type();

  // TODO: select MRM scenario from system function state machine

  if (current_scenario_->scenario_type() != pnc::ScenarioType::ROAD_CHANGE &&
      frame.DistToChangePoint() < FLAGS_road_change_in_distance_threshold) {
    scenario_type = pnc::ScenarioType::ROAD_CHANGE;
  }
  if (current_scenario_->scenario_type() == pnc::ScenarioType::ROAD_CHANGE &&
      frame.DistToChangePoint() > FLAGS_road_change_out_distance_threshold) {
    scenario_type = pnc::ScenarioType::LANE_FOLLOW;
  }

  ADEBUG << "Current scenario: "
         << pnc::ScenarioType_Name(current_scenario_->scenario_type());
  if (current_scenario_->scenario_type() != scenario_type) {
    current_scenario_ = CreateScenario(scenario_type);
  }
}

void ScenarioManager::UpdatePlanningInternal(
    const Frame& frame, const pnc::ScenarioType& scenario_type) {
  
}

bool ScenarioManager::RegisterScenarios() {
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file, 
                              &scenario_config_map_[pnc::ScenarioType::LANE_FOLLOW]));
  ACHECK(Scenario::LoadConfig(FLAGS_scenario_road_change_config_file, 
                              &scenario_config_map_[pnc::ScenarioType::ROAD_CHANGE]));
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_emergency_pull_over_config_file,
      &scenario_config_map_[pnc::ScenarioType::EMERGENCY_PULL_OVER]));
  ACHECK(Scenario::LoadConfig(
      FLAGS_scenario_emergency_stop_config_file,
      &scenario_config_map_[pnc::ScenarioType::EMERGENCY_STOP]));
  return true;
}

std::unique_ptr<Scenario> ScenarioManager::CreateScenario(
    const pnc::ScenarioType& scenario_type) {
  std::unique_ptr<Scenario> ptr;
  switch(scenario_type) {
    case pnc::ScenarioType::LANE_FOLLOW:
      ptr.reset(new LaneFollowScenario(scenario_config_map_[scenario_type], internal_));
      break;
    case pnc::ScenarioType::ROAD_CHANGE:
      ptr.reset(new RoadChangeScenario(scenario_config_map_[scenario_type], internal_));
      break;
    case pnc::ScenarioType::EMERGENCY_STOP:
      ptr.reset(new EmergencyStopScenario(scenario_config_map_[scenario_type],
                                          internal_));
      break;
    case pnc::ScenarioType::EMERGENCY_PULL_OVER:
      ptr.reset(new EmergencyPullOverScenario(scenario_config_map_[scenario_type],
                                          internal_));
      break;
    default:
      return nullptr;
  }

  if (ptr != nullptr) {
    ptr->Init();
  }

  return ptr;
}

} // namespace planning
} // namespace xju
