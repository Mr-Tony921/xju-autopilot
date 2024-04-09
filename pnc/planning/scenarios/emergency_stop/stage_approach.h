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

class EmergencyStopStageApproach : public Stage {
 public:
  EmergencyStopStageApproach(const pnc::StageConfig& config,
                             std::shared_ptr<PlanningInternal>& internal);

  Stage::Status Process(Frame* frame) override;

 private:
  Stage::Status FinishStage();
  
 private:
  std::unique_ptr<LaneDecider> lane_decider_ptr_;
};

} // namespace planning
} // namespace xju
