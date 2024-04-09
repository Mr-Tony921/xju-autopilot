/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/scenarios/stage.h"
#include "planning/scenarios/common/lane_decider.h"
#include "planning.pb.h"
#include "planning_status.pb.h"

namespace xju {
namespace planning {

class EmergencyPullOverStageLaneChange : public Stage {
 public:
  EmergencyPullOverStageLaneChange(const pnc::StageConfig& config,
                             std::shared_ptr<PlanningInternal>& internal);

  Stage::Status Process(Frame* frame) override;

 private:
  Stage::Status FinishStage();
  void UpdateStatus(const LaneChangeStatus::Status status_code,
                    const double timestamp,
                    const std::string& path_id);
  
 private:
  std::unique_ptr<LaneDecider> lane_decider_ptr_;
};

} // namespace planning
} // namespace xju
