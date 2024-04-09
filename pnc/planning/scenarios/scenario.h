/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "planning/scenarios/stage.h"
#include "planning/common/frame/frame.h"
#include "planning/common/planning_internal/planning_internal.h"
#include "scenario_config.pb.h"
#include "stage_config.pb.h"

namespace xju {
namespace planning {

class Scenario {
 public:
  enum Status {
    UNKNOWN = 0,
    PROCESSING = 1,
    FINISHED = 2,
  };

 public:
  Scenario() = delete;
  virtual ~Scenario() = default;

  bool Init();

  Scenario(const pnc::ScenarioConfig& config,
           std::shared_ptr<PlanningInternal>& internal);

  static bool LoadConfig(const std::string& config_file, pnc::ScenarioConfig* config);

  pnc::ScenarioType scenario_type() const {
    return config_.scenario_type();
  }

  Status status() const {
    return scenario_status_;
  }

  const std::string& name() const {
    return name_;
  }

  virtual Status Process(Frame* frame);

 protected:
  virtual std::unique_ptr<Stage> CreateStage(
      const pnc::StageConfig& stage_config, std::shared_ptr<PlanningInternal>& internal) = 0;

 protected:
  std::string name_;
  pnc::ScenarioConfig config_;
  Status scenario_status_ = Status::UNKNOWN;
  std::unordered_map<pnc::StageType, pnc::StageConfig, std::hash<int>> stage_config_map_;
  std::unordered_map<pnc::StageType, std::shared_ptr<Stage>> stage_map_;
  std::unique_ptr<Stage> current_stage_;
  std::shared_ptr<PlanningInternal> internal_;
};

} // namespace planning
} // namespace xju
