/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>

#include "planning/scenarios/stage.h"
#include "planning/scenarios/common/lane_decider.h"
#include "planning.pb.h"
#include "planning_status.pb.h"

namespace xju {
namespace planning {

class RoadChangeStage : public Stage {
 public:
  RoadChangeStage(const pnc::StageConfig& config,
                  std::shared_ptr<PlanningInternal>& internal);
  
  Status Process(Frame* frame) override;

 private:
  void UpdateStatus(const LaneChangeStatus::Status status_code,
                    const double timestamp,
                    const std::string& path_id);

  DiscretizedTrajectory GenerateEmptyTrajectory(const double start_time) const;

 private:
  std::unique_ptr<LaneDecider> lane_decider_ptr_;
};

} // namespace planning
} // namespace xju
