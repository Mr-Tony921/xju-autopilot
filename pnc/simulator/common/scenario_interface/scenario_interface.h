/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "simulator/common/scenario_interface/scenario.h"
#include "simulator/common/tinyxml/tinyxml2.h"

namespace xju {
namespace simulator {

class ScenarioInterface {
 public:
  ScenarioInterface() { scenario_ = std::make_shared<Scenario>(); };
  ~ScenarioInterface() = default;

  static std::shared_ptr<ScenarioInterface> GetInstance() {
    static auto instance = std::make_shared<ScenarioInterface>();
    return instance;
  }

  bool InitScenario(const char* file_path);

  const auto& GetLanelets() { return scenario_->lanelets; };

 private:
  std::shared_ptr<Scenario> scenario_;
};

}  // namespace simulator
}  // namespace xju
