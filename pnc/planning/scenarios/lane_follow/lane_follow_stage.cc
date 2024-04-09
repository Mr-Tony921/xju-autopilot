/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/lane_follow/lane_follow_stage.h"

#include "common/time/time.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

LaneFollowStage::LaneFollowStage(
    const pnc::StageConfig& config,
    std::shared_ptr<PlanningInternal>& internal)
    : Stage(config, internal) {
  if (config.has_lane_decider_config()) {
    lane_decider_ptr_.reset(new LaneDecider(config_.lane_decider_config(), internal_));
  } else {
    lane_decider_ptr_.reset(new LaneDecider(internal_));
  }
}

Stage::Status LaneFollowStage::Process(Frame* frame) {
  Stage::Process(frame);
  bool lane_decider_status = lane_decider_ptr_->Process(frame);
  if(!lane_decider_status) {
    return Stage::Status::ERROR;
  }
  auto reference_line_infos = frame->GetPrioritizedReferenceLineInfos();
  const double now_time = frame->planning_start_time();
  for (auto& reference_line_info : reference_line_infos) {
    bool status = ProcessTaskOnReferenceLine(reference_line_info, frame);
    if (internal_->planning_status().lane_change_status().status() == LaneChangeStatus::LANE_CHANGE && 
        !status) {
      ADEBUG << "update status lane_change to lane_follow";
      UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now_time, reference_line_info->path_id());
      continue;
    }
    if (status) {
      return Stage::Status::PROCESSING;
    } else {
      UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now_time, reference_line_info->path_id());
      return Stage::Status::ERROR;
    }
  }
  return Stage::Status::ERROR;
}

void LaneFollowStage::UpdateStatus(
    const LaneChangeStatus::Status status_code,
    const double timestamp,
    const std::string& path_id) {
  LaneChangeStatus* lane_change_status = internal_->mutable_planning_status()
                                             ->mutable_lane_change_status();
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);
}

} // namespace planning
} // namespace xju