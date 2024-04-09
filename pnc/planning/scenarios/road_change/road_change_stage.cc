/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/planning_gflags/planning_gflags.h"
#include "planning/common/util/util.h"
#include "planning/scenarios/road_change/road_change_stage.h"

#include "common/time/time.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

RoadChangeStage::RoadChangeStage(
    const pnc::StageConfig& config,
    std::shared_ptr<PlanningInternal>& internal)
    : Stage(config, internal) {
  if (config.has_lane_decider_config()) {
    lane_decider_ptr_.reset(new LaneDecider(config_.lane_decider_config(), internal_));
  } else {
    lane_decider_ptr_.reset(new LaneDecider(internal_));
  }
}

Stage::Status RoadChangeStage::Process(Frame* frame) {
  Stage::Process(frame);
  internal_->mutable_planning_status()->clear_path_point_to_pass_through();
  auto lane_ids = frame->LanesToChangePoint();
  auto change_point_dist = frame->DistToChangePoint();
  auto target_lane_id = frame->ego_lane_id();
  auto lanes_to_target = 0;
  auto near = false;
  const double lane_segment = 80;
  if (lane_ids.empty() || frame->ego_lane_id() == lane_ids.back()) {
    ADEBUG << "RoadChange : no need to change, clear lane_ids";
    frame->ClearLanesToChangePoint();
  } else {
    auto ego_iter =
        std::find(lane_ids.begin(), lane_ids.end(), frame->ego_lane_id());
    if (ego_iter != lane_ids.end() && ego_iter != lane_ids.end() - 1) {
      target_lane_id = *std::next(ego_iter);
    }
    lanes_to_target = std::distance(ego_iter, lane_ids.end() - 1);
    near = lanes_to_target > 0 &&
           change_point_dist <= lane_segment * lanes_to_target;
  }
  if (!lane_decider_ptr_->Process(target_lane_id, near, frame)) {
    return Stage::Status::ERROR;
  }
  auto reference_line_infos = frame->GetPrioritizedReferenceLineInfos();
  if (near) {
    auto current_lane_remain =
        change_point_dist - lane_segment * (lanes_to_target - 1);
    if (current_lane_remain < 0) {
      AERROR << "RoadChange : missing : lanes_to_target : " << lanes_to_target
             << " current_lane_remain : " << current_lane_remain;
      // TODO(tony): what action should be done here?
    }

    auto self_ref_line_info = reference_line_infos.back();
    pnc::Vec2d vehicle_xy(frame->vehicle_state().x(),
                          frame->vehicle_state().y());
    pnc::SLPoint vehicle_sl;
    const double vehicle_s =
        self_ref_line_info->reference_line().XYToSL(vehicle_xy, &vehicle_sl)
            ? vehicle_sl.s() : 25;
    const double stop_margin = 10;
    const std::string stop_id =
        "ROADCHANGE_VIR_" + self_ref_line_info->reference_line().id();
    auto stop_s = std::max(0.0, vehicle_s + current_lane_remain - stop_margin);
    if (-1 == planning::util::BuildStopDecision(stop_id, stop_s, 0.0, frame,
                                                self_ref_line_info)) {
      AERROR << "RoadChange : BuildStopDecision error! id " << stop_id;
    }
    auto speed = std::min(
        5.55, self_ref_line_info->reference_line().GetSpeedLimitFromS(stop_s));
    self_ref_line_info->set_cruise_speed(speed);

    if (reference_line_infos.size() > 1) {
      auto target_ref_line_info = reference_line_infos.front();
      auto through_s = stop_s + stop_margin;
      const auto& through_point =
          target_ref_line_info->reference_line().GetReferencePoint(through_s);
      auto through = internal_->mutable_planning_status()
                         ->mutable_path_point_to_pass_through();
      through->set_x(through_point.x());
      through->set_y(through_point.y());
      through->set_theta(through_point.theta());
      speed = std::min(
          FLAGS_road_change_cruise_speed,
          target_ref_line_info->reference_line().GetSpeedLimitFromS(through_s));
      target_ref_line_info->set_cruise_speed(speed);
    }
  }
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
      if (near) {
        AINFO << "RoadChange : publish empty trajectory";
        auto special_ref = reference_line_infos.front();
        special_ref->set_trajectory(GenerateEmptyTrajectory(
            frame->planning_start_point().relative_time()));
        special_ref->set_trajectory_type(ADCTrajectory::TRAJECTORY_FALLBACK);
        special_ref->set_drivable(true);
        special_ref->set_cost(0.0);
        return Stage::Status::PROCESSING;
      }
      return Stage::Status::ERROR;
    }
  }
  return Stage::Status::ERROR;
}

void RoadChangeStage::UpdateStatus(
    const LaneChangeStatus::Status status_code,
    const double timestamp,
    const std::string& path_id) {
  LaneChangeStatus* lane_change_status = internal_->mutable_planning_status()
                                             ->mutable_lane_change_status();
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);
}

DiscretizedTrajectory RoadChangeStage::GenerateEmptyTrajectory(
    const double start_time) const {
  static DiscretizedTrajectory empty_trajectory;
  static bool init = false;
  if (!init) {
    AINFO << "RoadChange : generate empty trajectory!";
    init = true;
    pnc::PathPoint path_point;
    path_point.set_x(0.0);
    path_point.set_y(0.0);
    path_point.set_theta(0.0);
    path_point.set_kappa(0.0);
    path_point.set_dkappa(0.0);
    path_point.set_ddkappa(0.0);
    path_point.set_s(0.0);
    pnc::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(0.0);
    trajectory_point.set_a(0.0);
    for (int i = 0; i <= 8; ++i) {
      trajectory_point.set_relative_time(1.0 * i);
      empty_trajectory.AppendTrajectoryPoint(trajectory_point);
    }
  }

  std::for_each(empty_trajectory.begin(), empty_trajectory.end(),
                [&start_time](pnc::TrajectoryPoint& pt) {
                  pt.set_relative_time(start_time + pt.relative_time());
                });

  return empty_trajectory;
}

} // namespace planning
} // namespace xju