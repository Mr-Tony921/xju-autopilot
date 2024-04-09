/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "planning/scenarios/stage.h"
#include "planning.pb.h"
#include "planning_status.pb.h"

namespace xju {
namespace planning {
  
class EmergencyPullOverStageSlowDown : public Stage {
 public:
  EmergencyPullOverStageSlowDown(const pnc::StageConfig& config,
                             std::shared_ptr<PlanningInternal>& internal);

  Status Process(Frame* frame) override;
 
 private:
  Stage::Status FinishStage();
};

} // namespace planning
} // namespace xju
