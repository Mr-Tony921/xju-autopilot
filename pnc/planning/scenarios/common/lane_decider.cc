/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/common/lane_decider.h"

#include "common/file/file.h"
#include "common/logger/logger.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace xju {
namespace planning {

LaneDecider::LaneDecider(std::shared_ptr<PlanningInternal>& internal)
    : internal_(internal) {
  pnc::File::GetProtoConfig<LaneDeciderConfig>(
      FLAGS_lane_decider_default_config_file, &config_);
  load_default_config_ = true;
  vehicle_config_ = pnc::VehicleConfigProvider::GetConfig();
  lane_change_speed_decider_ptr_.reset(new LaneChangeSpeedDecider(config_));
}

LaneDecider::LaneDecider(const LaneDeciderConfig& config,
                         std::shared_ptr<PlanningInternal>& internal)
    : internal_(internal) {
  if (!load_default_config_) {
    pnc::File::GetProtoConfig<LaneDeciderConfig>(
        FLAGS_lane_decider_default_config_file, &config_);
    load_default_config_ = true;
  }
  config_.MergeFrom(config);
  vehicle_config_ = pnc::VehicleConfigProvider::GetConfig();
  lane_change_speed_decider_ptr_.reset(new LaneChangeSpeedDecider(config_));
}

bool LaneDecider::Process(Frame* const frame) {
  ADEBUG << "lane is not allowed, update status lane_change_prepare to "
            "lane_follow";
  auto* lane_change_status =
      internal_->mutable_planning_status()->mutable_lane_change_status();
  auto reference_line_infos = frame->GetReferenceLineInfos();
  ADEBUG << "reference_line_infos.size() = " << reference_line_infos.size();
  auto cur_refrence_line_info = frame->GetCurrentReferenceLineInfo();
  if (reference_line_infos.empty() || !cur_refrence_line_info) {
    AERROR
        << "Reference_infos empty or cur_reference_line_info is not existed.";
    return false;
  }
  LaneDeciderInit(frame);
  double now = pnc::Time::NowInSeconds();
  static bool is_first_run = true;
  if (is_first_run) {
    if (!lane_change_status->has_status()) {
      ADEBUG << "lane_change_status has no status()";
      UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
    }
    is_first_run = false;
  }

  if (lane_change_status->status() == LaneChangeStatus::LANE_FOLLOW) {
    ADEBUG << "lane follow status";
    if (!IsOnNonRightRoad(frame)) {
      non_right_lane_time_ = pnc::Time::NowInSeconds();
    }
    lane_change_status->set_turn_signal(pnc::TurnSignal::TURN_NONE);
    if (IsNeedChangeLane(frame) && CheckLaneByDirection(frame) &&
        (CheckSpaceByDirection(frame) || SelectSpaceByDirection(frame))) {
      UpdateStatus(LaneChangeStatus::LANE_CHANGE_PREPARE, now,
                   frame->ego_lane_id());
      ADEBUG << "update status lane_follow to lane_change_prepare";
    }
  } else if (lane_change_status->status() ==
             LaneChangeStatus::LANE_CHANGE_PREPARE) {
    ADEBUG << "lane prepare status";

    if (IsNeedChangeLane(frame)) {
      if (CheckLaneByDirection(frame)) {
        if (CheckSpaceByDirection(frame)) {
          if (now - lane_change_status->timestamp() >
              config_.lane_change_prepare_keep_time()) {
            ADEBUG << "update status lane_change_prepare to lane_change";
            change_direction_ = pre_desision_direction_;
            no_need_change_counter_ = 0;
            UpdateStatus(LaneChangeStatus::LANE_CHANGE, now,
                         frame->ego_lane_id());
          }
        } else {
          static int select_space_failure_counter = 0;
          if (SelectSpaceByDirection(frame)) {
            if (now - lane_change_status->timestamp() >
                config_.lane_change_max_prepare_keep_time()) {
              no_need_change_counter_ = 0;
              select_space_failure_counter = 0;
              UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                           frame->ego_lane_id());
              pre_desision_direction_ = LaneChangeDirection::FORWARD;
              ADEBUG << "select space process is timeout, update status "
                        "lane_change_prepare to lane_follow ";
            }
          } else {
            select_space_failure_counter++;
          }
          if (select_space_failure_counter >
              config_.select_space_failure_counter()) {
            select_space_failure_counter = 0;
            UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                         frame->ego_lane_id());
            pre_desision_direction_ = LaneChangeDirection::FORWARD;
            ADEBUG << "select space process can't select available space in "
                      "time, update status lane_change_prepare to lane_follow ";
          }
        }
      } else {
        no_need_change_counter_ = 0;
        ADEBUG << "lane is not allowed, update status lane_change_prepare to "
                  "lane_follow";
        UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
      }
    } else {
      ADEBUG << "lane change need is not exsited in prerapre, update status "
                "lane_change_prepare to lane_follow";
      UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
    }
  } else if (lane_change_status->status() == LaneChangeStatus::LANE_CHANGE) {
    ADEBUG << "lane change status";
    double lane_change_sucess_judge_dis = last_left_lane_width_;
    if (change_direction_ == LaneChangeDirection::LEFT) {
      lane_change_sucess_judge_dis =
          last_left_lane_width_ - config_.lane_change_sucess_judge_buffer();
    } else if (change_direction_ == LaneChangeDirection::RIGHT) {
      lane_change_sucess_judge_dis =
          last_right_lane_width_ - config_.lane_change_sucess_judge_buffer();
    }
    ADEBUG << "last bumper l = " << last_front_bumper_sl_.l();
    ADEBUG << "lane_change_sucess_judge_dis = " << lane_change_sucess_judge_dis;
    if (std::fabs(last_front_bumper_sl_.l()) < lane_change_sucess_judge_dis) {
      if (IsNeedChangeLane(frame)) {
        if (!CheckLaneByDirection(frame)) {
          UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                       frame->ego_lane_id());
          pre_desision_direction_ = LaneChangeDirection::FORWARD;
          change_direction_ = LaneChangeDirection::FORWARD;
          ADEBUG << "update status lane_change to lane_follow, lane change "
                    "failure, lane condition is not content";
        }
        if (!CheckSpaceByDirection(frame)) {
          UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                       frame->ego_lane_id());
          pre_desision_direction_ = LaneChangeDirection::FORWARD;
          change_direction_ = LaneChangeDirection::FORWARD;
          ADEBUG << "update status lane_change to lane_follow, lane change "
                    "failure, space  is not content";
        }
        if (IsOnNonRightRoad(frame)) {
          non_right_lane_time_ = pnc::Time::NowInSeconds();
        }
      } else {
        UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
        pre_desision_direction_ = LaneChangeDirection::FORWARD;
        change_direction_ = LaneChangeDirection::FORWARD;
        ADEBUG
            << "update status lane_change to lane_follow, lane change need is "
               "not exsited in lane_change, space  is not content";
      }
    } else {
      ADEBUG << "lane_change sucess, not check lane and space, lane_change "
                "state still in lane_change";
      if (cur_refrence_line_info->reference_line().id() !=
          lane_change_status->path_id()) {
        UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
        pre_desision_direction_ = LaneChangeDirection::FORWARD;
        change_direction_ = LaneChangeDirection::FORWARD;
        ADEBUG
            << "update status lane_change to lane_follow, lane change sucess";
      }
    }
  }
  last_left_lane_width_ = left_lane_width_;
  last_right_lane_width_ = right_lane_width_;
  last_front_bumper_sl_.set_l(front_bumper_sl_.l());
  ADEBUG << "pre_desision_direction_ = "
         << static_cast<int>(pre_desision_direction_);
  ADEBUG << "change_direction_ = " << static_cast<int>(change_direction_);
  bool set_prioritized_reference_status = false;
  if (lane_change_status->status() == LaneChangeStatus::LANE_CHANGE) {
    set_prioritized_reference_status =
        SetPrioritizedReferenceLineInfos(change_direction_, frame);
  } else {
    set_prioritized_reference_status =
        SetPrioritizedReferenceLineInfos(LaneChangeDirection::FORWARD, frame);
  }
  if (!set_prioritized_reference_status) {
    return false;
  }
  return true;
}

bool LaneDecider::Process(const std::string& target_lane_id,
                          const bool ignore_prepare,
                          Frame* const frame) {
  auto* lane_change_status =
      internal_->mutable_planning_status()->mutable_lane_change_status();
  auto reference_line_infos = frame->GetReferenceLineInfos();
  auto cur_refrence_line_info = frame->GetCurrentReferenceLineInfo();
  if (reference_line_infos.empty() || !cur_refrence_line_info) {
    AERROR << "RoadChange Stage: Reference lines error";
    return false;
  }
  LaneDeciderInit(frame);
  double now = pnc::Time::NowInSeconds();
  static bool is_first_run = true;
  if (is_first_run) {
    if (!lane_change_status->has_status()) {
      UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
    }
    is_first_run = false;
  }

  static int lane_change_stay_counter = 0;
  // const int stay_threshold = ignore_prepare ? 10 : std::numeric_limits<int>::max();
  auto left_lane_id = left_reference_line_info_
                          ? left_reference_line_info_->reference_line().id()
                          : "LNA";
  auto right_lane_id = right_reference_line_info_
                           ? right_reference_line_info_->reference_line().id()
                           : "RNA";
  const int stay_threshold = std::numeric_limits<int>::max();
  switch (lane_change_status->status()) {
    case LaneChangeStatus::LANE_FOLLOW:
      lane_change_stay_counter = 0;
      if (target_lane_id == frame->ego_lane_id()) {
        ADEBUG << "RoadChange Stage: target lane equal with ego, keep forward";
        UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
      } else if (target_lane_id != left_lane_id && target_lane_id != right_lane_id) {
        ADEBUG << "RoadChange Stage: target lane not reachable, keep forward";
        UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
      } else {
        pre_desision_direction_ = target_lane_id == left_lane_id
                                      ? LaneChangeDirection::LEFT
                                      : LaneChangeDirection::RIGHT;
        if (CheckLaneByDirection(frame)) {
          if (CheckSpaceByDirection(frame)) {
            ADEBUG << "RoadChange Stage: update status to lane_change";
            change_direction_ = pre_desision_direction_;
            UpdateStatus(LaneChangeStatus::LANE_CHANGE, now,
                         frame->ego_lane_id());
          } else if (!ignore_prepare && SelectSpaceByDirection(frame)) {
            ADEBUG << "RoadChange Stage: update status to lane_change_prepare";
            UpdateStatus(LaneChangeStatus::LANE_CHANGE_PREPARE, now,
                         frame->ego_lane_id());
          } else {
            ADEBUG << "RoadChange Stage: condition not satisfied, keep forward";
            UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                         frame->ego_lane_id());
          }
        } else {
          ADEBUG << "RoadChange Stage: condition not satisfied, keep forward";
          UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                       frame->ego_lane_id());
        }
      }
      break;
    case LaneChangeStatus::LANE_CHANGE_PREPARE:
      lane_change_stay_counter = 0;
      if (CheckLaneByDirection(frame)) {
        if (CheckSpaceByDirection(frame)) {
          if (ignore_prepare || now - lane_change_status->timestamp() >
                                    config_.lane_change_prepare_keep_time()) {
            ADEBUG << "update status lane_change_prepare to lane_change";
            change_direction_ = pre_desision_direction_;
            no_need_change_counter_ = 0;
            UpdateStatus(LaneChangeStatus::LANE_CHANGE, now,
                         frame->ego_lane_id());
          }
        } else {
          static int select_space_failure_counter = 0;
          if (SelectSpaceByDirection(frame)) {
            if (now - lane_change_status->timestamp() >
                config_.lane_change_max_prepare_keep_time()) {
              no_need_change_counter_ = 0;
              select_space_failure_counter = 0;
              UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                           frame->ego_lane_id());
              pre_desision_direction_ = LaneChangeDirection::FORWARD;
              ADEBUG << "select space process is timeout, update status "
                        "lane_change_prepare to lane_follow";
            }
          } else {
            select_space_failure_counter++;
          }
          if (select_space_failure_counter >
              config_.select_space_failure_counter()) {
            select_space_failure_counter = 0;
            UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                         frame->ego_lane_id());
            pre_desision_direction_ = LaneChangeDirection::FORWARD;
            ADEBUG << "select space process can't select available space in "
                      "time, update status "
                      "lane_change_prepare to lane_follow";
          }
        }
      } else {
        no_need_change_counter_ = 0;
        ADEBUG << "lane is not allowed, update status lane_change_prepare to "
                  "lane_follow";
        UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
      }
      break;
    case LaneChangeStatus::LANE_CHANGE: {
      ++lane_change_stay_counter;
      auto lane_change_sucess_judge_dis =
          (change_direction_ == LaneChangeDirection::LEFT
               ? last_left_lane_width_
               : last_right_lane_width_) -
          config_.lane_change_sucess_judge_buffer();
      if (std::fabs(last_front_bumper_sl_.l()) < lane_change_sucess_judge_dis) {
        if ((!CheckLaneByDirection(frame) || !CheckSpaceByDirection(frame)) &&
            lane_change_stay_counter < stay_threshold) {
          UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                       frame->ego_lane_id());
          pre_desision_direction_ = LaneChangeDirection::FORWARD;
          change_direction_ = LaneChangeDirection::FORWARD;
        }
      } else {
        if (frame->ego_lane_id() == target_lane_id) {
          UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now,
                       frame->ego_lane_id());
          pre_desision_direction_ = LaneChangeDirection::FORWARD;
          change_direction_ = LaneChangeDirection::FORWARD;
          ADEBUG << "RoadChange Stage: lane change sucess";
        }
      }
      break;
    }
    default:
      lane_change_stay_counter = 0;
      ADEBUG << "RoadChange Stage: lane_change_status->status() error ";
      break;
  }
  last_left_lane_width_ = left_lane_width_;
  last_right_lane_width_ = right_lane_width_;
  last_front_bumper_sl_.set_l(front_bumper_sl_.l());
  ADEBUG << "RoadChange Stage: "
         << " pre_desision_direction_ = "
         << static_cast<int>(pre_desision_direction_)
         << " change_direction_ = " << static_cast<int>(change_direction_)
         << " lane_change_status after judgement = "
         << lane_change_status->status();
  bool set_prioritized_reference_status = false;
  if (lane_change_status->status() == LaneChangeStatus::LANE_CHANGE) {
    set_prioritized_reference_status =
        SetPrioritizedReferenceLineInfos(change_direction_, frame);
  } else {
    set_prioritized_reference_status =
        SetPrioritizedReferenceLineInfos(LaneChangeDirection::FORWARD, frame);
  }

  return set_prioritized_reference_status;
}

void LaneDecider::GetNearObstaclesInfo(Frame* const frame) {
  current_lane_front_obs_index_ = -1;
  current_lane_back_obs_index_ = -1;
  left_lane_front_obs_index_ = -1;
  left_lane_back_obs_index_ = -1;
  right_lane_front_obs_index_ = -1;
  right_lane_back_obs_index_ = -1;

  // Init distance between obstacle and ego.
  current_lane_closest_obs_dist_front_ = std::numeric_limits<float>::max();
  current_lane_closest_obs_dist_back_ = std::numeric_limits<float>::max();
  left_lane_closest_obs_dist_front_ = std::numeric_limits<float>::max();
  left_lane_closest_obs_dist_back_ = std::numeric_limits<float>::max();
  right_lane_closest_obs_dist_front_ = std::numeric_limits<float>::max();
  right_lane_closest_obs_dist_back_ = std::numeric_limits<float>::max();

  if (frame->obstacles().empty()) {
    return;
  }
  ADEBUG
      << "obstacles size ="
      << cur_reference_line_info_->path_decision()->obstacles().Items().size();
  float ego_start_s = cur_reference_line_info_->car_sl_boundary().start_s();
  float ego_end_s = cur_reference_line_info_->car_sl_boundary().end_s();
  int i = 0;
  ADEBUG << "ego_start_s = " << ego_start_s;
  ADEBUG << "ego_end_s = " << ego_end_s;
  left_lane_obstacles_.clear();
  right_lane_obstacles_.clear();
  std::pair<std::array<double, 3>, double> obs_state;
  for (const auto* iter : frame->obstacles()) {
    i++;
    // judge if obstacle is in current lane
    SLBoundary obstacle_sl_boundary_cur, obstacle_sl_boundary_left,
        obstacle_sl_boundary_right;
    ADEBUG << "current reference line id = "
           << cur_reference_line_info_->reference_line().id();
    Obstacle* obstacle_in_cur_ref =
        cur_reference_line_info_->path_decision()->Find(iter->id());
    if (obstacle_in_cur_ref) {
      obstacle_sl_boundary_cur = obstacle_in_cur_ref->sl_boundary();
    }
    Obstacle* obstacle_in_left_ref = nullptr;
    if (left_reference_line_info_) {
      obstacle_in_left_ref =
          left_reference_line_info_->path_decision()->Find(iter->id());
      obstacle_sl_boundary_left = obstacle_in_left_ref->sl_boundary();
    }

    Obstacle* obstacle_in_right_ref = nullptr;
    if (right_reference_line_info_) {
      obstacle_in_right_ref =
          right_reference_line_info_->path_decision()->Find(iter->id());
      obstacle_sl_boundary_right = obstacle_in_right_ref->sl_boundary();
    }
    pnc::SLPoint obstacle_sl;
    pnc::Vec2d obstacle_xy{iter->perception_obstacle().position().x(),
                           iter->perception_obstacle().position().y()};
    cur_reference_line_info_->reference_line().XYToSL(obstacle_xy,
                                                      &obstacle_sl);

    if (obstacle_in_cur_ref &&
        cur_reference_line_info_->reference_line().IsOnLane(
            obstacle_sl_boundary_cur)) {
      UpdateDecisionObsIndex(
          i, vehicle_sl_, obstacle_sl, ego_start_s, ego_end_s,
          obstacle_sl_boundary_cur, &current_lane_closest_obs_dist_front_,
          &current_lane_closest_obs_dist_back_, &current_lane_front_obs_index_,
          &current_lane_back_obs_index_);
    }

    if (left_reference_line_info_ && obstacle_in_left_ref &&
        left_reference_line_info_->reference_line().IsOnLane(
            obstacle_sl_boundary_left)) {
      UpdateDecisionObsIndex(
          i, vehicle_sl_, obstacle_sl, ego_start_s, ego_end_s,
          obstacle_sl_boundary_left, &left_lane_closest_obs_dist_front_,
          &left_lane_closest_obs_dist_back_, &left_lane_front_obs_index_,
          &left_lane_back_obs_index_);
      std::array<double, 3> state = {obstacle_sl.s(),
                                     obstacle_in_left_ref->speed(),
                                     obstacle_in_left_ref->acceleration()};
      obs_state = std::make_pair(
          state, obstacle_in_left_ref->perception_obstacle().length());
      left_lane_obstacles_.push_back(obs_state);
    }

    if (right_reference_line_info_ && obstacle_in_right_ref &&
        right_reference_line_info_->reference_line().IsOnLane(
            obstacle_sl_boundary_right)) {
      UpdateDecisionObsIndex(
          i, vehicle_sl_, obstacle_sl, ego_start_s, ego_end_s,
          obstacle_sl_boundary_right, &right_lane_closest_obs_dist_front_,
          &right_lane_closest_obs_dist_back_, &right_lane_front_obs_index_,
          &right_lane_back_obs_index_);
      std::array<double, 3> state = {obstacle_sl.s(),
                                     obstacle_in_right_ref->speed(),
                                     obstacle_in_right_ref->acceleration()};
      obs_state = std::make_pair(
          state, obstacle_in_right_ref->perception_obstacle().length());
      right_lane_obstacles_.push_back(obs_state);
    }
  }
  std::sort(left_lane_obstacles_.begin(), left_lane_obstacles_.end(),
            [](const std::pair<std::array<double, 3>, double>& left,
               const std::pair<std::array<double, 3>, double>& right) {
              return left.first[0] < right.first[0];
            });
  std::sort(right_lane_obstacles_.begin(), right_lane_obstacles_.end(),
            [](const std::pair<std::array<double, 3>, double>& left,
               const std::pair<std::array<double, 3>, double>& right) {
              return left.first[0] < right.first[0];
            });

  if (current_lane_front_obs_index_ >= 0) {
    const Obstacle* obs = frame->obstacles()[current_lane_front_obs_index_];
    if (obs) {
      pnc::SLPoint obstacle_sl;
      pnc::Vec2d obstacle_xy{obs->perception_obstacle().position().x(),
                             obs->perception_obstacle().position().y()};
      cur_reference_line_info_->reference_line().XYToSL(obstacle_xy,
                                                        &obstacle_sl);
      std::array<double, 3> state = {obstacle_sl.s(), obs->speed(),
                                     obs->acceleration()};
      ego_lane_front_closest_obs_ =
          std::make_pair(state, obs->perception_obstacle().length());
    }
  } else {
    std::array<double, 3> state = {0, 0, 0};
    ego_lane_front_closest_obs_ = std::make_pair(state, 0);
  }

  ADEBUG << "current_lane_closest_obs_dist_front_ = "
         << current_lane_closest_obs_dist_front_;
  ADEBUG << "current_lane_closest_obs_dist_back_ = "
         << current_lane_closest_obs_dist_back_;
  ADEBUG << "current_lane_front_obs_index_ = " << current_lane_front_obs_index_;
  ADEBUG << "current_lane_back_obs_index_ = " << current_lane_back_obs_index_;

  ADEBUG << "left_lane_closest_obs_dist_front_ = "
         << left_lane_closest_obs_dist_front_;
  ADEBUG << "left_lane_closest_obs_dist_back_ = "
         << left_lane_closest_obs_dist_back_;
  ADEBUG << "left_lane_front_obs_index_ = " << left_lane_front_obs_index_;
  ADEBUG << "left_lane_back_obs_index_ = " << left_lane_back_obs_index_;

  ADEBUG << "right_lane_closest_obs_dist_front_ = "
         << right_lane_closest_obs_dist_front_;
  ADEBUG << "right_lane_closest_obs_dist_back_ = "
         << right_lane_closest_obs_dist_back_;
  ADEBUG << "right_lane_front_obs_index_ = " << right_lane_front_obs_index_;
  ADEBUG << "right_lane_back_obs_index_ = " << right_lane_back_obs_index_;
}

bool LaneDecider::UpdateDecisionObsIndex(
    const int index, const pnc::SLPoint& ego_sl_point,
    const pnc::SLPoint& obstacle_sl_point, const double ego_start_s,
    const double ego_end_s, const SLBoundary& obstacle_slboundary,
    float* const closest_obs_dist_front, float* const closest_obs_dist_back,
    int* const closest_obs_front_index, int* const closest_obs_back_index) {
  bool is_obstacle_front =
      (obstacle_sl_point.s() > ego_sl_point.s()) ? true : false;
  double distance = 0;

  if (is_obstacle_front) {
    distance = obstacle_slboundary.start_s() - ego_end_s;
    if (distance < *closest_obs_dist_front) {
      *closest_obs_dist_front = distance;
      *closest_obs_front_index = index - 1;
    }
  } else {
    distance = ego_start_s - obstacle_slboundary.end_s();
    if (distance < *closest_obs_dist_back) {
      *closest_obs_dist_back = distance;
      *closest_obs_back_index = index - 1;
    }
  }
  return true;
}

// If static obstacle exists in self lane, it is possible for lanechange
bool LaneDecider::CheckAheadObsIsStatic(Frame* const frame,
                                        LaneChangeDirection* direction) {
  static uint64_t counter = 0;
  if (current_lane_front_obs_index_ >= 0) {
    auto* obst_ptr = frame->obstacles()[current_lane_front_obs_index_];

    double current_speed = cur_reference_line_info_->vehicle_state()
                               .linear_velocity();
    double distance_thr = 10;
    distance_thr = current_speed * current_speed *
                   config_.static_obstacle_dis_thr_factor();
    ADEBUG << "static_distance_thr  = " << distance_thr;
    if (distance_thr < config_.min_dis_thr()) {
      distance_thr = config_.min_dis_thr();
    } else if (distance_thr > config_.max_dis_thr()) {
      distance_thr = config_.max_dis_thr();
    }
    ADEBUG << "static_distance_thr with limit = " << distance_thr;
    if (obst_ptr) {
      ADEBUG << "obst_ptr->speed() = " << obst_ptr->speed();
      if (obst_ptr->speed() < config_.static_obstacle_speed_threshold() &&
          (current_lane_closest_obs_dist_front_ < distance_thr)) {
        counter++;
      } else {
        counter = 0;
      }
      if (counter > config_.obs_speed_counter_thr()) {
        counter = config_.obs_speed_counter_thr();
        if (IsChangLaneforLeftRoad(frame) && left_reference_line_info_) {
          *direction = LaneChangeDirection::LEFT;
          return true;
        } else if (right_reference_line_info_) {
          *direction = LaneChangeDirection::RIGHT;
          return true;
        } else {
          return false;
        }
      } else {
        return false;
      }
    } else {
      AWARN << "front static obstacle is null, need to check";
      counter = 0;
      return false;
    }
  } else {
    ADEBUG << "front static obstacle is not existed";
    counter = 0;
    return false;
  }
}

bool LaneDecider::CheckAheadObsSpeedIsLow(Frame* const frame,
                                          LaneChangeDirection* direction) {
  static uint64_t counter = 0;
  double obstacle_v = 0;
  if (current_lane_front_obs_index_ >= 0) {
    double current_speed =
        cur_reference_line_info_->vehicle_state().linear_velocity();
    const double target_cruise_speed = cur_reference_line_info_->cruise_speed();
    double judge_speed = std::fmax(current_speed, target_cruise_speed);
    auto* obst_ptr = frame->obstacles()[current_lane_front_obs_index_];
    if (obst_ptr) {
      obstacle_v = obst_ptr->speed();
      ADEBUG << "obstacle_v: = " << obstacle_v
             << " target_cruise_speed: = " << target_cruise_speed
             << " current_speed: = " << current_speed
             << "  counter: = " << counter
             << " current_lane_closest_obs_dist_front_: = "
             << current_lane_closest_obs_dist_front_;
      ADEBUG << "obs_speed_ratio = " << config_.obs_speed_ratio();
      ADEBUG << "judge_speed = " << judge_speed;
      ADEBUG << "obstacle_v = " << obst_ptr->speed();
      ADEBUG << "current_speed = " << current_speed;
      double distance_thr = 30;
      if (current_speed > obstacle_v) {
        distance_thr = (current_speed - obstacle_v) *
                           (current_speed - obstacle_v) *
                           config_.relative_speed_dis_thr_factor() +
                       current_speed * config_.ego_speed_dis_thr_factor1();
      } else {
        distance_thr = current_speed * config_.ego_speed_dis_thr_factor2();
      }
      ADEBUG << "distance_thr = " << distance_thr;
      if (distance_thr < config_.min_dis_thr()) {
        distance_thr = config_.min_dis_thr();
      } else if (distance_thr > config_.max_dis_thr()) {
        distance_thr = config_.max_dis_thr();
      }
      ADEBUG << "distance_thr with limit = " << distance_thr;
      if ((obst_ptr->speed() * config_.obs_speed_ratio() < judge_speed) &&
          (current_lane_closest_obs_dist_front_ < distance_thr)) {
        counter++;
      }
    } else {
      AWARN << "front static obstacle is null, need check";
    }
  } else {
    counter = 0;
  }
  if (counter >= config_.obs_speed_counter_thr()) {
    ADEBUG << "low speed obstacle acturally exists ";
    counter = config_.obs_speed_counter_thr();
    JudgeTrafficEfficiency(obstacle_v, frame, direction);
    if (LaneChangeDirection::FORWARD != *direction) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}

// Judge if it has enough space for lanechange in adjacent lane.
bool LaneDecider::CheckNearLaneFrontAndBackSpace(
    const Frame& frame, const int closest_obs_front_index,
    const int closest_obs_back_index, const double closest_obs_dist_front,
    const double closest_obs_dist_back) {
  ADEBUG << "obst front: = " << closest_obs_dist_front
         << ", obst back: = " << closest_obs_dist_back;

  double dist_buffer = 0.0;
  if (internal_->planning_status().lane_change_status().status() ==
      LaneChangeStatus::LANE_CHANGE) {
    dist_buffer = -config_.space_hysteresis_buffer();
  } else {
    dist_buffer = config_.space_hysteresis_buffer();
  }

  if (closest_obs_front_index >= 0) {
    double current_speed =
        frame.vehicle_state().linear_velocity();
    const Obstacle* front_obstacle_ptr =
        frame.obstacles()[closest_obs_front_index];
    if (!front_obstacle_ptr) {
      return false;
    }
    ADEBUG << "current speed:" << current_speed;
    double th_max_dist_front =
        current_speed * config_.max_safe_distance_front_time() +
        config_.min_obst_dist_front() + dist_buffer;
    double th_min_dist_front =
        current_speed * config_.min_safe_distance_front_time() +
        config_.min_obst_dist_front() + dist_buffer;
    ADEBUG << "th_max_dist_front = " << th_max_dist_front;
    ADEBUG << "th_min_dist_front = " << th_min_dist_front;
    if (closest_obs_dist_front < th_min_dist_front) {
      AERROR << "Front obstacle is too near!";
      return false;
    }

    // front obs dist is not far(middle far), check speed is satisfied
    if (closest_obs_dist_front < th_max_dist_front) {
      double min_safe_speed_ratio = 1.0 - config_.far_safe_speed_diff_ratio();
      double max_safe_speed_ratio = 1.0 + config_.near_safe_speed_diff_ratio();
      double v_obj_front_judge = 0;
      if (current_speed > 0) {
        v_obj_front_judge = (current_speed * max_safe_speed_ratio -
                             current_speed * min_safe_speed_ratio) /
                                (th_max_dist_front - th_min_dist_front) *
                                (th_max_dist_front - closest_obs_dist_front) +
                            current_speed * min_safe_speed_ratio;
      }
      ADEBUG << "v_obj_front_judge = " << v_obj_front_judge;
      ADEBUG << "front_obstacle_v = " << front_obstacle_ptr->speed();
      if (front_obstacle_ptr->speed() < v_obj_front_judge) {
        AERROR
            << "front obstacle speed is too low, lane change is not allowed.";
        return false;
      }
    }
  }

  if (closest_obs_back_index >= 0) {
    const Obstacle* back_obstacle_ptr =
        frame.obstacles()[closest_obs_back_index];
    if (!back_obstacle_ptr) {
      return false;
    }

    double current_speed =
        frame.vehicle_state().linear_velocity();
    double th_max_dist_back =
        back_obstacle_ptr->is_static()
            ? config_.back_static_obstacle_safe_dis()
            : (current_speed * config_.max_safe_distance_back_time() +
               config_.min_obst_dist_back()) +
                  dist_buffer;
    double th_min_dist_back =
        back_obstacle_ptr->is_static()
            ? config_.back_static_obstacle_safe_dis()
            : current_speed * config_.min_safe_distance_back_time() +
                  config_.min_obst_dist_back() + dist_buffer;
    ADEBUG << "th_max_dist_back = " << th_max_dist_back;
    ADEBUG << "th_min_dist_back = " << th_min_dist_back;
    if (closest_obs_dist_back < th_min_dist_back) {
      AERROR << "Back obstacle is too near!";
      return false;
    }

    // back obs is not far(middle far), check speed is satisfied
    if (closest_obs_dist_back < th_min_dist_back) {
      double min_safe_speed_ratio = 1.0 - config_.near_safe_speed_diff_ratio();
      double max_safe_speed_ratio = 1.0 + config_.far_safe_speed_diff_ratio();
      double v_obj_back_judge = 0;
      if (current_speed > 0) {
        v_obj_back_judge =
            back_obstacle_ptr->is_static()
                ? 0
                : (current_speed * min_safe_speed_ratio -
                   current_speed * max_safe_speed_ratio) /
                          (th_max_dist_back - th_min_dist_back) *
                          (th_max_dist_back - closest_obs_dist_back) +
                      current_speed * max_safe_speed_ratio;
      }
      ADEBUG << "v_obj_back_judge = " << v_obj_back_judge;
      ADEBUG << "back_obstacle_v = " << back_obstacle_ptr->speed();
      if (back_obstacle_ptr->speed() > v_obj_back_judge) {
        AERROR << "back obstacle speed too fast, lane change is not allowed.";
        return false;
      }
    }
  }
  return true;
}

bool LaneDecider::PreLaneChangeDesision(Frame* const frame,
                                        LaneChangeDirection* direction) {
  if (config_.enable_lane_change_ego_speed() && !CheckEgoSpeedSatisfied()) {
    return false;
  }

  const auto& emergency_pull_over_status =
      internal_->planning_status().emergency_pull_over_status();
  if (emergency_pull_over_status.lane_change_right() &&
      cur_reference_line_info_->reference_line().lane_order() !=
          pnc::LaneOrder::FIRST_LANE && right_reference_line_info_) {
    *direction = LaneChangeDirection::RIGHT;
    ADEBUG << "Emergency pull over, need to change lane";
    return true;
  }

  if (config_.enable_lane_change_non_right_road() &&
      CheckLongTimeOnNonRightRoad(frame, direction)) {
    ADEBUG << "On non-right road, need to change lane";
    return true;
  }

  if (config_.enable_lane_change_static_obstacle() &&
      (CheckAheadObsIsStatic(frame, direction))) {
    ADEBUG << "Static obstacle is front, need to change lane,direction is = "
           << static_cast<int>(*direction);
    return true;
  }

  if (config_.enable_lane_change_low_speed_obstacle() &&
      CheckAheadObsSpeedIsLow(frame, direction)) {
    ADEBUG << "Ahead low_speed osbtacle, need to change lane, direction is = "
           << static_cast<int>(*direction);
    return true;
  }
  return false;
}

bool LaneDecider::IsNeedChangeLane(Frame* const frame) {
  auto* lane_change_status =
      internal_->mutable_planning_status()->mutable_lane_change_status();
  LaneChangeDirection direction = LaneChangeDirection::FORWARD;
  if (PreLaneChangeDesision(frame, &direction)) {
    no_need_change_counter_ = 0;
    pre_desision_direction_ = direction;
    return true;
  } else {
    if (lane_change_status->status() == LaneChangeStatus::LANE_CHANGE_PREPARE ||
        lane_change_status->status() == LaneChangeStatus::LANE_CHANGE) {
      no_need_change_counter_++;
      ADEBUG << "No need lane change, delay couter is = "
             << no_need_change_counter_;
      if (no_need_change_counter_ < config_.no_need_change_counter_thr()) {
        return true;
      }
    }
  }
  pre_desision_direction_ = direction;
  return false;
}
bool LaneDecider::CheckLaneTypeSatisfied(
    const LaneChangeDirection& direction) const {
  if (left_reference_line_info_ && direction == LaneChangeDirection::LEFT) {
    pnc::RestrictedLaneType type;
    left_reference_line_info_->reference_line().GetRestrictedLaneType(
        vehicle_sl_.s(), type);
    if (!left_reference_line_info_->reference_line().is_death_lane() &&
        !left_reference_line_info_->reference_line().is_emergency_lane() &&
        type == pnc::RestrictedLaneType::RESTRICTED_LANE_TYPE_NONE) {
      return true;
    } else {
      ADEBUG << "Left lanetype is not available for lanechange";
      return false;
    }
  }

  if (right_reference_line_info_ && direction == LaneChangeDirection::RIGHT) {
    pnc::RestrictedLaneType type;
    right_reference_line_info_->reference_line().GetRestrictedLaneType(
        vehicle_sl_.s(), type);
    if (!right_reference_line_info_->reference_line().is_death_lane() &&
        !right_reference_line_info_->reference_line().is_emergency_lane() &&
        type == pnc::RestrictedLaneType::RESTRICTED_LANE_TYPE_NONE) {
      return true;
    } else {
      ADEBUG << "Right lanetype is not available for lanechange";
      return false;
    }
  }
  return false;
}

bool LaneDecider::CheckLaneMarkSatisfied(
    const LaneChangeDirection& direction) const {
  const pnc_map::ReferenceLine cur_reference_line =
      cur_reference_line_info_->reference_line();
  pnc::LaneMarkingType left_lane_marker, right_lane_marker;
  cur_reference_line.GetLaneMarkingType(vehicle_sl_.s(), left_lane_marker,
                                        right_lane_marker);
  ADEBUG << "direction = " << static_cast<int>(direction);

  if (direction == LaneChangeDirection::LEFT) {
    if (left_lane_marker ==
            pnc::LaneMarkingType::LBR_MARKING_SINGLE_DASHED_LINE ||
        left_lane_marker == pnc::LaneMarkingType::
                                LBR_MARKING_LEFT_SOLID_LINE_RIGHT_DASHED_LINE) {
      return true;
    } else {
      ADEBUG << "left_lane_mark is not DASHED_LINE";
      return false;
    }
  } else if (direction == LaneChangeDirection::RIGHT) {
    if (right_lane_marker ==
            pnc::LaneMarkingType::LBR_MARKING_SINGLE_DASHED_LINE ||
        right_lane_marker ==
            pnc::LaneMarkingType::
                LBR_MARKING_LEFT_SOLID_LINE_RIGHT_DASHED_LINE) {
      return true;
    } else {
      ADEBUG << "right_lane_mark is not DASHED_LINE";
      return false;
    }
  } else {
    ADEBUG << "error, target_direction is FORWARD";
    return false;
  }
}

bool LaneDecider::CheckEgoSpeedSatisfied() const {
  ADEBUG << "cur_vehicle_speed = "
         << cur_reference_line_info_->vehicle_state().linear_velocity();
  if (cur_reference_line_info_->vehicle_state().linear_velocity() <
      config_.ego_speed_limit_lane_change()) {
    return true;
  } else {
    AINFO << "cur_vehicle_speed is high and can't change lane";
    return false;
  }
}

// judge traffic flow efficience for possible lane_change .
void LaneDecider::JudgeTrafficEfficiency(const double front_obstacle_v,
                                         Frame* const frame,
                                         LaneChangeDirection* direction) {
  ADEBUG << "front_obstacle_v = " << front_obstacle_v;
  ADEBUG << "left_lane_front_obs_index_ = " << left_lane_front_obs_index_;
  ADEBUG << "right_lane_front_obs_index_ = " << right_lane_front_obs_index_;
  bool left_ok = false;
  if (left_reference_line_info_) {
    if (left_lane_front_obs_index_ >= 0) {
      const Obstacle* left_obst_ptr =
          frame->obstacles()[left_lane_front_obs_index_];
      ADEBUG << "left efficiency: left obs speed: " << left_obst_ptr->speed();
      if (left_obst_ptr) {
        if (left_obst_ptr->speed() >=
            front_obstacle_v * config_.traffic_efficiency_ratio()) {
          left_ok = true;
        } else {
          ADEBUG << "changelane to left is not allowed by left_front obstacle";
          left_ok = false;
        }
      } else {
        AERROR << "left_front obstacle aquirement is not ok, please check: ";
        left_ok = false;
      }
    } else {
      AWARN << "left low speed obstacle is real null in "
               "JudgeTrafficEfficiencyFun";
      left_ok = true;
    }
  } else {
    left_ok = false;
  }
  ADEBUG << "left_ok = " << left_ok;
  if (left_reference_line_info_) {
    if (!IsChangLaneforLeftRoad(frame)) {
      left_ok = false;
    }
  } else {
    left_ok = false;
  }

  bool right_ok = false;
  if (right_reference_line_info_) {
    if (right_lane_front_obs_index_ >= 0) {
      const Obstacle* right_obst_ptr =
          frame->obstacles()[right_lane_front_obs_index_];
      if (right_obst_ptr) {
        ADEBUG << "traffic_efficiency_ratio = "
               << config_.traffic_efficiency_ratio();
        if (right_obst_ptr->speed() >=
            front_obstacle_v * config_.traffic_efficiency_ratio()) {
          right_ok = true;
        } else {
          ADEBUG
              << "changelane to right is not allowed by right_front obstacle";
          right_ok = false;
        }
      } else {
        AERROR << "right_front obstacle aquirement is not ok, please check: ";
        right_ok = false;
      }
    } else {
      right_ok = true;
      AWARN << "right low speed obstacle is null in JudgeTrafficEfficiencyFun";
    }
  } else {
    right_ok = false;
  }
  ADEBUG << "left_ok = " << left_ok;
  ADEBUG << "right_ok = " << right_ok;
  if (left_ok && !right_ok && left_reference_line_info_) {
    *direction = LaneChangeDirection::LEFT;
  } else if (!left_ok && right_ok && right_reference_line_info_) {
    *direction = LaneChangeDirection::RIGHT;
  } else {
    *direction = LaneChangeDirection::FORWARD;
  }
}

bool LaneDecider::CheckLaneBend(const Frame& frame) const {
  double average_kappa = 0.0;
  for (const auto& iter :
       cur_reference_line_info_->reference_line().reference_points()) {
    average_kappa += std::fabs(iter.kappa());
  }

  if (0.0 == config_.curv_radius_thr() ||
      cur_reference_line_info_->reference_line().reference_points().size() ==
          0) {
    AERROR << "config_.curv_radius is zero!";
    return false;
  }

  average_kappa /=
      cur_reference_line_info_->reference_line().reference_points().size();
  if (average_kappa > 1.0 / config_.curv_radius_thr()) {
    ADEBUG << "Curve kappa not satisfied:" << average_kappa
           << " > req:" << 1.0 / config_.curv_radius_thr();
    return false;
  } else {
    return true;
  }
}

bool LaneDecider::CheckLaneChangeEnd(Frame* frame) {
  auto reference_lne_infos = frame->GetReferenceLineInfos();
  pnc::Vec2d vehicle_xy{frame->planning_start_point().path_point().x(),
                      frame->planning_start_point().path_point().y()};
  pnc::SLPoint vehicle_sl;
  (reference_lne_infos).front()->reference_line().XYToSL(vehicle_xy, &vehicle_sl);

  if (vehicle_sl.l() < config_.lane_change_l_thr()) {
    auto reference_line_info_priority =
        frame->GetPrioritizedReferenceLineInfos();
    auto iter = (reference_line_info_priority).begin();
    while (iter != reference_line_info_priority.end()) {
      if ((*iter)->is_change_lane()) {
        reference_line_info_priority.erase(iter);
      }
      iter++;
    }
    return true;
  }
  return false;
}

bool LaneDecider::SelectSpaceByDirection(Frame* const frame) {
  change_direction_ = LaneChangeDirection::FORWARD;
  std::array<double, 3> feasible_state{0.0, vehicle_state_[1], 0.0};
  LaneChangeStatus* lane_change_status =
      internal_->mutable_planning_status()->mutable_lane_change_status();
  lane_change_status->set_prepare_acceleration((feasible_state).at(0));
  lane_change_status->set_prepare_speed((feasible_state).at(1));
  lane_change_status->set_prepare_time((feasible_state).at(2));
  if (!config_.enable_lane_change_speed_decider()) {
    return false;
  }

  if ( /* current_lane_front_obs_index_ >= 0 */ true) {
    if (pre_desision_direction_ == LaneChangeDirection::LEFT) {
      if (lane_change_speed_decider_ptr_->Process(
              cur_reference_line_info_->cruise_speed(), left_lane_obstacles_,
              ego_lane_front_closest_obs_, vehicle_state_, &feasible_state)) {
        lane_change_status->set_prepare_acceleration((feasible_state).at(0));
        lane_change_status->set_prepare_speed((feasible_state).at(1));
        lane_change_status->set_prepare_time((feasible_state).at(2));
        ADEBUG << "select left space is available";
        return true;
      } else {
        ADEBUG << "select left space is not available";
        return false;
      }
    } else if (pre_desision_direction_ == LaneChangeDirection::RIGHT) {
      if (lane_change_speed_decider_ptr_->Process(
              cur_reference_line_info_->cruise_speed(), right_lane_obstacles_,
              ego_lane_front_closest_obs_, vehicle_state_, &feasible_state)) {
        lane_change_status->set_prepare_acceleration((feasible_state).at(0));
        lane_change_status->set_prepare_speed((feasible_state).at(1));
        lane_change_status->set_prepare_time((feasible_state).at(2));
        ADEBUG << "select right space is available";
        return true;
      } else {
        ADEBUG << "select right space is not available";
        return false;
      }
    } else {
      return true;
    }
  } else {
    return false;
  }
}

bool LaneDecider::CheckSpaceByDirection(Frame* const frame) {
  if (pre_desision_direction_ == LaneChangeDirection::LEFT) {
    if (CheckNearLaneFrontAndBackSpace(*frame, left_lane_front_obs_index_,
                                       left_lane_back_obs_index_,
                                       left_lane_closest_obs_dist_front_,
                                       left_lane_closest_obs_dist_back_)) {
      ADEBUG << "left space is ok";
      return true;
    } else {
      ADEBUG << "left space is not ok";
      return false;
    }
  } else if (pre_desision_direction_ == LaneChangeDirection::RIGHT) {
    if (CheckNearLaneFrontAndBackSpace(*frame, right_lane_front_obs_index_,
                                       right_lane_back_obs_index_,
                                       right_lane_closest_obs_dist_front_,
                                       right_lane_closest_obs_dist_back_)) {
      ADEBUG << "right space is ok";
      return true;
    } else {
      ADEBUG << "right space is not ok";
      return false;
    }
  } else {
    return true;
  }
}

bool LaneDecider::CheckLaneByDirection(Frame* const frame) {
  if (pre_desision_direction_ == LaneChangeDirection::LEFT) {
    if (CheckLaneMarkSatisfied(pre_desision_direction_) &&
        CheckLaneTypeSatisfied(pre_desision_direction_) &&
        CheckLaneBend(*frame)) {
      ADEBUG << "left lane is ok";
      return true;
    } else {
      ADEBUG << "left lane is not ok";
      return false;
    }
  } else if (pre_desision_direction_ == LaneChangeDirection::RIGHT) {
    if (CheckLaneMarkSatisfied(pre_desision_direction_) &&
        CheckLaneTypeSatisfied(pre_desision_direction_) &&
        CheckLaneBend(*frame)) {
      ADEBUG << "right lane is ok";
      return true;
    } else {
      ADEBUG << "right lane is not ok";
      return false;
    }
  } else {
    return true;
  }
}
bool LaneDecider::SetPrioritizedReferenceLineInfos(
    const LaneChangeDirection& direction, Frame* const frame) {
  std::list<std::shared_ptr<ReferenceLineInfo>> reference_line_infos_priority;
  if (!cur_reference_line_info_) {
    AERROR << "Current reference_line is not existed";
    return false;
  }
  double now = pnc::Time::NowInSeconds();
  if (direction == LaneChangeDirection::FORWARD) {
    ADEBUG << "forward, only one refrence_line";
    reference_line_infos_priority.push_back(cur_reference_line_info_);
  } else if (direction == LaneChangeDirection::LEFT) {
    ADEBUG << "Left reference_line_info need to be added";
    if (left_reference_line_info_) {
      reference_line_infos_priority.push_back(left_reference_line_info_);
    } else {
      AWARN << "Left reference_line is not existed";
      UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
    }
    reference_line_infos_priority.push_back(cur_reference_line_info_);
  } else if (direction == LaneChangeDirection::RIGHT) {
    if (right_reference_line_info_) {
      reference_line_infos_priority.push_back(right_reference_line_info_);
    } else {
      AWARN << "Right reference_line is not existed";
      UpdateStatus(LaneChangeStatus::LANE_FOLLOW, now, frame->ego_lane_id());
    }
    reference_line_infos_priority.push_back(cur_reference_line_info_);
  }
  int i = 0;
  for (auto ref : reference_line_infos_priority) {
    ADEBUG << "Priority(" << i << ")"
           << "reference_line_info id = " << ref->reference_line().id()
           << " output";
    i++;
  }
  frame->SetPrioritizedReferenceLineInfos(reference_line_infos_priority);
  return true;
}

bool LaneDecider::CheckLongTimeOnNonRightRoad(Frame* const frame,
                                              LaneChangeDirection* direction) {
  double now = pnc::Time::NowInSeconds();
  if (IsOnNonRightRoad(frame) && right_reference_line_info_ &&
      (now - non_right_lane_time_) > config_.non_right_lane_running_time()) {
    *direction = LaneChangeDirection::RIGHT;
    ADEBUG << "Vehicle is on non-right road for a long time";
    return true;
  } else {
    return false;
  }
}

// TODO:reference_line nums will depend on map interface.
// It will be modified in future;
bool LaneDecider::IsOnNonRightRoad(Frame* const frame) const {
  const auto reference_line_infos = frame->GetReferenceLineInfos();
  pnc::LaneOrder cur_lane_order =
      cur_reference_line_info_->reference_line().lane_order();
  if ((reference_line_infos).size() <= 3) {
    if (cur_lane_order == pnc::LaneOrder::FIRST_LANE) {
      return false;
    } else {
      ADEBUG << "Vehicle is on non-right road,road_size <= 3";
      return true;
    }
  } else if ((reference_line_infos).size() >= 4) {
    if (cur_lane_order == pnc::LaneOrder::FIRST_LANE ||
        cur_lane_order == pnc::LaneOrder::SECOND_LANE) {
      return false;
    } else {
      ADEBUG << "Vehicle is on non-right road road_size >= 4";
      return true;
    }
  }
  return false;
}

// TODO:reference_line nums will depend on map interface.
// It will be modified in future;
bool LaneDecider::IsChangLaneforLeftRoad(Frame* const frame) const {
  const auto reference_line_infos = frame->GetReferenceLineInfos();
  pnc::LaneOrder cur_lane_order =
      cur_reference_line_info_->reference_line().lane_order();
  ADEBUG << "Cur_lane_order = " << cur_lane_order;
  if ((reference_line_infos).size() <= 1) {
    ADEBUG << "There is no left lane, lane change is not allowed";
    return false;
  } else if ((reference_line_infos).size() <= 4) {
    if (cur_lane_order == pnc::LaneOrder::FIRST_LANE) {
      ADEBUG << "Ego is on rightmost road,allow to change left";
      return true;
    } else {
      return false;
    }
  }
  return false;
}

void LaneDecider::UpdateStatus(const LaneChangeStatus::Status& status_code,
                               const double timestamp,
                               const std::string& path_id) {
  LaneChangeStatus* lane_change_status =
      internal_->mutable_planning_status()->mutable_lane_change_status();
  lane_change_status->set_timestamp(timestamp);
  lane_change_status->set_path_id(path_id);
  lane_change_status->set_status(status_code);

  if (status_code == LaneChangeStatus::LANE_FOLLOW) {
    lane_change_status->set_turn_signal(pnc::TurnSignal::TURN_NONE);
  } else {
    if (pre_desision_direction_ == LaneChangeDirection::LEFT) {
      lane_change_status->set_turn_signal(pnc::TurnSignal::TURN_LEFT);
    } else if (pre_desision_direction_ == LaneChangeDirection::RIGHT) {
      lane_change_status->set_turn_signal(pnc::TurnSignal::TURN_RIGHT);
    } else {
      lane_change_status->set_turn_signal(pnc::TurnSignal::TURN_NONE);
    }
  }
}

void LaneDecider::LaneDeciderInit(Frame* const frame) {
  cur_reference_line_info_ = frame->GetCurrentReferenceLineInfo();
  left_reference_line_info_ = frame->GetLeftReferenceLineInfo();
  right_reference_line_info_ = frame->GetRightReferenceLineInfo();

  pnc::Vec2d vehicle_xy{frame->planning_start_point().path_point().x(),
                      frame->planning_start_point().path_point().y()};
  cur_reference_line_info_->reference_line().XYToSL(vehicle_xy, &vehicle_sl_);
  vehicle_state_ = {vehicle_sl_.s(), frame->planning_start_point().v(),
                  frame->planning_start_point().a()};

  double front_bumper_x =
      vehicle_config_.front_edge_to_center() +
      vehicle_config_.back_edge_to_center() -
      vehicle_config_.back_overhang_length();

  pnc::Vec2d vehicle_front_bumper_xy{
      frame->planning_start_point().path_point().x() + front_bumper_x,
      frame->planning_start_point().path_point().y()};
  cur_reference_line_info_->reference_line().XYToSL(vehicle_front_bumper_xy,
                                                    &front_bumper_sl_);

  if (!cur_reference_line_info_->reference_line().GetLaneWidth(
          front_bumper_sl_.s(), left_lane_width_, right_lane_width_)) {
    left_lane_width_ = 3.75 / 2.0;
    right_lane_width_ = 3.75 / 2.0;
    AERROR << "Failed to get lane width ";
  }
  GetNearObstaclesInfo(frame);
}
}  // namespace planning
}  // namespace xju
