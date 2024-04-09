/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/time/time.h"
#include "lane_change_speed_decider.h"
#include "lane_decider_config.pb.h"
#include "planning/common/frame/frame.h"
#include "planning/common/planning_internal/planning_internal.h"
#include "vehicle_config.pb.h"

namespace xju {
namespace planning {
enum class LaneChangeDirection { FORWARD = 0, LEFT = 1, RIGHT = 2 };

class LaneDecider {
 public:
  LaneDecider(std::shared_ptr<PlanningInternal>& internal);
  LaneDecider(const LaneDeciderConfig& config,
              std::shared_ptr<PlanningInternal>& internal);
  ~LaneDecider() = default;
  bool Process(Frame* const frame);
  bool Process(const std::string& target_lane_id,
               const bool ignore_prepare,
               Frame* const frame);

 private:
  void LaneDeciderInit(Frame* const frame);
  void UpdateStatus(const LaneChangeStatus::Status& status_code,
                    const double timestamp, const std::string& path_id);
  void GetNearObstaclesInfo(Frame* const frame);
  bool UpdateDecisionObsIndex(const int index, const pnc::SLPoint& ego_sl_point,
                              const pnc::SLPoint& obstacle_sl_point,
                              const double ego_start_s, const double ego_end_s,
                              const SLBoundary& obstacle_slboundary,
                              float* const closest_obs_dist_front,
                              float* const closest_obs_dist_back,
                              int* const closest_obs_front_index,
                              int* const closest_obs_back_index);
  bool IsNeedChangeLane(Frame* const frame);
  bool PreLaneChangeDesision(Frame* const frame,
                             LaneChangeDirection* direction);
  bool CheckAheadObsIsStatic(Frame* const frame,
                             LaneChangeDirection* direction);
  bool CheckLaneTypeSatisfied(const LaneChangeDirection& direction) const;
  bool CheckLaneMarkSatisfied(const LaneChangeDirection& direction) const;
  bool CheckEgoSpeedSatisfied() const;
  bool CheckAheadObsSpeedIsLow(Frame* const frame,
                               LaneChangeDirection* direction);
  void JudgeTrafficEfficiency(const double front_obstacle_v, Frame* const frame,
                              LaneChangeDirection* direction);
  bool CheckLaneBend(const Frame& frame) const;
  bool CheckNearLaneFrontAndBackSpace(const Frame& frame,
                                      const int closest_obs_front_index,
                                      const int closest_obs_back_index,
                                      const double closest_obs_dist_front,
                                      const double closest_obs_dist_back);
  bool CheckSpaceByDirection(Frame* const frame);
  bool CheckLaneByDirection(Frame* const frame);
  bool SelectSpaceByDirection(Frame* const frame);
  bool CheckLaneChangeEnd(Frame* frame);
  bool CheckLongTimeOnNonRightRoad(Frame* const frame,
                                   LaneChangeDirection* direction);
  bool LaneChangeJudge(Frame* const frame);
  bool IsOnNonRightRoad(Frame* const frame) const;
  bool IsChangLaneforLeftRoad(Frame* const frame) const;
  bool SetPrioritizedReferenceLineInfos(const LaneChangeDirection& direction,
                                        Frame* const frame);
  bool load_default_config_ = false;
  LaneDeciderConfig config_;
  std::shared_ptr<PlanningInternal> internal_;
  pnc::VehicleConfig vehicle_config_;
  double non_right_lane_time_ = pnc::Time::NowInSeconds();

  LaneChangeDirection change_direction_ = LaneChangeDirection::FORWARD;
  LaneChangeDirection pre_desision_direction_ = LaneChangeDirection::FORWARD;

  int current_lane_front_obs_index_ = -1;
  int current_lane_back_obs_index_ = -1;
  int left_lane_front_obs_index_ = -1;
  int left_lane_back_obs_index_ = -1;
  int right_lane_front_obs_index_ = -1;
  int right_lane_back_obs_index_ = -1;

  float current_lane_closest_obs_dist_front_;
  float current_lane_closest_obs_dist_back_;
  float left_lane_closest_obs_dist_front_;
  float left_lane_closest_obs_dist_back_;
  float right_lane_closest_obs_dist_front_;
  float right_lane_closest_obs_dist_back_;

  int no_need_change_counter_ = 0;
  std::vector<std::pair<std::array<double, 3>, double>> left_lane_obstacles_;
  std::vector<std::pair<std::array<double, 3>, double>> right_lane_obstacles_;
  std::pair<std::array<double, 3>, double> ego_lane_front_closest_obs_;
  std::array<double, 3> vehicle_state_;
  pnc::SLPoint vehicle_sl_;
  pnc::SLPoint front_bumper_sl_;
  pnc::SLPoint last_front_bumper_sl_;
  double left_lane_width_ = 3.75 / 2.0;
  double right_lane_width_ = 3.75 / 2.0;
  double last_left_lane_width_ = 3.75 / 2.0;
  double last_right_lane_width_ = 3.75 / 2.0;
  std::shared_ptr<ReferenceLineInfo> cur_reference_line_info_;
  std::shared_ptr<ReferenceLineInfo> left_reference_line_info_;
  std::shared_ptr<ReferenceLineInfo> right_reference_line_info_;
  std::unique_ptr<LaneChangeSpeedDecider> lane_change_speed_decider_ptr_;
};

}  // namespace planning
}  // namespace xju
