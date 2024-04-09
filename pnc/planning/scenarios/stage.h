/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <map>
#include <memory>
#include <string>

#include "planning/tasks/task.h"
#include "planning/common/frame/frame.h"
#include "planning/common/reference_line_info/reference_line_info.h"
#include "planning/common/planning_internal/planning_internal.h"
#include "planning/common/speed/speed_data.h"
#include "planning.pb.h"
#include "pnc_point.pb.h"
#include "scenario_config.pb.h"
#include "stage_config.pb.h"
#include "task_config.pb.h"

namespace xju {
namespace planning {

class Stage {
 public:
  enum Status {
    UNKNOWN = 0,
    ERROR = 1,
    READY = 2,
    PROCESSING = 3,
    NEXT = 4,
    FINISHED = 5,
  };

 public:
  Stage() = delete;
  virtual ~Stage() = default;

  Stage(const pnc::StageConfig& config, 
        const std::shared_ptr<PlanningInternal>& internal);

  virtual Status Process(Frame* frame) {
    internal_->mutable_planning_status()->mutable_scenario_status()
        ->set_stage_type(config_.stage_type());
    return Status::PROCESSING;
  }

  pnc::StageType stage_type() const {
    return config_.stage_type();
  }

  Status status() const {
    return status_;
  }

  pnc::StageType next_stage_type() const {
    return next_stage_type_;
  }

  const std::string& name() const {
    return name_;
  }

  void SetScenarioConfig(const pnc::ScenarioConfig* const scenario_config) {
    scenario_config_ = scenario_config;
  }

 protected:
  bool ProcessTaskOnReferenceLine(std::shared_ptr<ReferenceLineInfo> const reference_line_info, Frame* frame);

  bool ProcessTaskOnOpenSpace(Frame* frame);

  void RecordDebugInfo(std::shared_ptr<ReferenceLineInfo> reference_line_info,
                       const std::string& name, const double time_diff_ms);

  virtual Stage::Status FinishScenario();

 protected:
  std::string name_;
  pnc::StageConfig config_;
  const pnc::ScenarioConfig* scenario_config_ = nullptr;
  Status status_ = Status::UNKNOWN;
  std::map<pnc::TaskType, std::shared_ptr<Task>> task_map_;
  std::vector<std::shared_ptr<Task>> task_list_;
  pnc::StageType next_stage_type_;
  std::shared_ptr<PlanningInternal> internal_;
};

} // namespace planning
} // namespace xju
