/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/deciders/lateral_shift_decider/lateral_shift_decider.h"

#include "common/logger/logger.h"
#include "common/math/math_utils.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning_task_config.pb.h"
namespace xju {
namespace planning {

LateralShiftDecider::LateralShiftDecider(
    const pnc::TaskConfig& config,
    const std::shared_ptr<PlanningInternal>& internal)
    : Task(config, internal) {}

void LateralShiftDecider::Init(const pnc::TaskConfig& config) {
  config_ = config;
}

void LateralShiftDecider::Reset() {}

bool LateralShiftDecider::Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info,
                                  Frame* const frame) {
  if (frame == nullptr) {
    ADEBUG << " frame is null";
    return false;
  }

  if (reference_line_info == nullptr) {
    ADEBUG << " reference_line_info is null";
    return false;
  }
  // 1. Status checks
  if (!internal_->planning_status().has_lateral_shift_status()) {
    ADEBUG << "There is no lateral_shift_status.";
    UpdateLateralShiftStatus("", LateralShiftStatus::NONE, 0.0);
  }
  auto lane_change_status = internal_->planning_status().lane_change_status();

  if (lane_change_status.status() == LaneChangeStatus::LANE_CHANGE) {
    ADEBUG << "Lane Changing. Can not Shift.";
    UpdateLateralShiftStatus("", LateralShiftStatus::NONE, 0.0);
    return true;
  }

  if (reference_line_info->is_change_lane()) {
    ADEBUG << "Reference lane is not current lane.";
    UpdateLateralShiftStatus("", LateralShiftStatus::NONE, 0.0);
    return true;
  }

  // 2. Update Adc state
  if (!UpdateADCState(*reference_line_info, *frame)) {
    AERROR << "Failed to update Adc state.";
    UpdateLateralShiftStatus("", LateralShiftStatus::NONE, 0.0);
    return true;
  }

  if (IsOutOfLane(*reference_line_info)) {
    ADEBUG << "Ego Box is out of current lane.";
    UpdateLateralShiftStatus("", LateralShiftStatus::NONE, 0.0);
    return true;    
  }

  // 3. Go through every obstacle and preprocess it.
  double left_distance_to_boundary = std::numeric_limits<double>::max();
  double right_distance_to_boundary = std::numeric_limits<double>::max();
  GetDistanceBetweenObstacleToBoundary(*reference_line_info,
                                       &left_distance_to_boundary,
                                       &right_distance_to_boundary);

  // 4. Calculate left and right shift distances
  double left_candidate_shift_distance =
      CalculateShiftDistances(left_distance_to_boundary);
  double right_candidate_shift_distance =
      CalculateShiftDistances(right_distance_to_boundary);
  ADEBUG << "left_candidate_shift_distance = " << left_candidate_shift_distance
        << ", right_candidate_shift_distance = "
        << right_candidate_shift_distance;

  // 5. Select shift direction
  LateralShiftStatus::Type candidate_type = LateralShiftStatus::NONE;
  double candidate_shift_distance = 0.0;

  SelectShiftDirection(left_candidate_shift_distance,
                       right_candidate_shift_distance, &candidate_type,
                       &candidate_shift_distance);

  ADEBUG << "shift direction is " << candidate_type << ", shift distance is "
         << candidate_shift_distance;
  ADEBUG << " Before ShiftArbitration shift direction is "
         << internal_->planning_status().lateral_shift_status().type()
         << ", shift distance is "
         << internal_->planning_status().lateral_shift_status().distance_m();
  // 7.Update status
  ShiftArbitration(candidate_type, candidate_shift_distance);
  AINFO << " After ShiftArbitration shift direction is "
        << internal_->planning_status().lateral_shift_status().type()
        << ", shift distance is "
        << internal_->planning_status().lateral_shift_status().distance_m();
  return true;
}

void LateralShiftDecider::ShiftArbitration(
    const LateralShiftStatus::Type& candidate_type,
    const double& candidate_shift_distance) {
  const auto& decider_config = config_.lateral_shift_decider_config();
  auto lateral_shift_status =
      internal_->planning_status().lateral_shift_status();
  if (lateral_shift_status.type() == LateralShiftStatus::NONE &&
      candidate_type == LateralShiftStatus::NONE) {
    ADEBUG << "Current type and last type are none.";
    safe_count_ = 0;
    hold_on_direction_count_ = 0;
    hold_on_distance_count_ = 0;
    warnning_count_ = 0;
    UpdateLateralShiftStatus("", LateralShiftStatus::NONE, 0.0);
    return;
  } else if (lateral_shift_status.type() == LateralShiftStatus::NONE &&
             candidate_type != LateralShiftStatus::NONE) {
    safe_count_ = 0;
    hold_on_direction_count_ = 0;
    hold_on_distance_count_ = 0;
    ++warnning_count_;
    ADEBUG << "Last type is none, but current type is not none. warnning_count_ : " << warnning_count_;
    if (warnning_count_ > decider_config.warnning_count()) {
      UpdateLateralShiftStatus(target_id_, candidate_type, candidate_shift_distance);
      ADEBUG << "New target obstacle appers, shift distance("
            << candidate_shift_distance << "), shift direction("
            << candidate_type << ")";
      warnning_count_ = 0;
    }
    return;
  } else if (lateral_shift_status.type() != LateralShiftStatus::NONE &&
             candidate_type == LateralShiftStatus::NONE) {
    hold_on_direction_count_ = 0;
    hold_on_distance_count_ = 0;
    warnning_count_ = 0;
    ++safe_count_;
    ADEBUG << "Last type is not none, but current type is none. safe_count_ : " << safe_count_;
    ADEBUG << "target_obstacle disappers, safe_count " << safe_count_;
    if (safe_count_ > decider_config.safe_count()) {
      UpdateLateralShiftStatus("", LateralShiftStatus::NONE, 0.0);
      ADEBUG << "Disable lateral shift.";
      safe_count_ = 0;
    }
    return;
  } else {
    ADEBUG << "Last type and current type are not none.";
    safe_count_ = 0;
    warnning_count_ = 0;
    if (candidate_type != lateral_shift_status.type()) {
      hold_on_distance_count_ = 0;
      ++hold_on_direction_count_;
      ADEBUG << "Shift direction changed, counting ... "
            << hold_on_direction_count_;
      if (hold_on_direction_count_ >
          decider_config.hold_on_direction_count()) {
        ADEBUG << "Change shift direction.";
        UpdateLateralShiftStatus(target_id_, candidate_type, candidate_shift_distance);
        hold_on_direction_count_ = 0;
      }
      return;
    }

    if (candidate_shift_distance < lateral_shift_status.distance_m()) {
      hold_on_direction_count_ = 0;
      ++hold_on_distance_count_;
      ADEBUG << "Shift distance changes from "
            << lateral_shift_status.distance_m() << " to "
            << candidate_shift_distance << ", counting ... "
            << hold_on_distance_count_;
      if (hold_on_distance_count_ > decider_config.hold_on_distance_count()) {
        ADEBUG << "Change shift distance(" << candidate_shift_distance << ")";
        UpdateLateralShiftStatus(target_id_, lateral_shift_status.type(),
                                 candidate_shift_distance);
        hold_on_distance_count_ = 0;
      }
    }
  }
}

void LateralShiftDecider::UpdateLateralShiftStatus(const std::string& id,
    const LateralShiftStatus::Type& type, const double& shift_distance) {
  ADEBUG << "Update LateralShiftStatus " << id << " " << type << " " << shift_distance;
  internal_->mutable_planning_status()
      ->mutable_lateral_shift_status()
      ->set_obstacle_id(id);
  internal_->mutable_planning_status()
      ->mutable_lateral_shift_status()
      ->set_type(type);
  internal_->mutable_planning_status()
      ->mutable_lateral_shift_status()
      ->set_distance_m(shift_distance);
}

void LateralShiftDecider::GetDistanceBetweenObstacleToBoundary(
    const ReferenceLineInfo& reference_line_info,
    double* left_distance_to_boundary, double* right_distance_to_boundary) {
  std::multimap<double, std::string> left_candidate_obstacles;
  std::multimap<double, std::string> right_candidate_obstacles;
  left_candidate_obstacles.insert(std::pair<double, std::string>(
      std::numeric_limits<double>::max(), ""));
  right_candidate_obstacles.insert(std::pair<double, std::string>(
      std::numeric_limits<double>::max(), ""));
  const auto& decider_config = config_.lateral_shift_decider_config();
  double lane_width_left = decider_config.default_lane_width() / 2.0;
  double lane_width_right = decider_config.default_lane_width() / 2.0;
  for (const auto* obstacle :
       reference_line_info.path_decision().obstacles().Items()) {
    const auto& obj_sl_boundary = obstacle->sl_boundary();
    if (obj_sl_boundary.end_s() < adc_frenet_s_) {
      ADEBUG << " Back obstacle (id:" << obstacle->id()
            << ", end_s:" << obj_sl_boundary.end_s() << ") is far from adc.";
      continue;
    } else if (obj_sl_boundary.start_s() >
               adc_frenet_s_ +
                   decider_config.forward_check_distance()) {  // 150
      ADEBUG << " Front obstacle (id:" << obstacle->id()
            << ", start_s:" << obj_sl_boundary.start_s()
            << ") is far from adc.";
      continue;
    }

    double mid_s = (obj_sl_boundary.start_s() + obj_sl_boundary.end_s()) / 2.0;
    if (!reference_line_info.reference_line().GetLaneWidth(
            mid_s, lane_width_left, lane_width_right)) {
      AERROR << "Failed to get lane width at s (" << mid_s << ")";
      lane_width_left = decider_config.default_lane_width() / 2.0;
      lane_width_right = decider_config.default_lane_width() / 2.0;
    }
    if (obj_sl_boundary.start_l() * obj_sl_boundary.end_l() < pnc::kMathEpsilon) {
      ADEBUG << "Obstacle is cross reference line.";
      continue;
    } else if (obj_sl_boundary.start_l() >
            lane_width_left + decider_config.lat_safe_buff() ||
        obj_sl_boundary.end_l() <
            -lane_width_right - decider_config.lat_safe_buff()) {  // 0.5
      ADEBUG << " Lateral obstacle (id:" << obstacle->id()
            << ", l:" << obj_sl_boundary.start_l() << "~"
            << obj_sl_boundary.end_l() << ") is far from adc.";
      continue;
    } else if ((obj_sl_boundary.start_l() > -lane_width_right &&
                obj_sl_boundary.start_l() < lane_width_left) ||
               (obj_sl_boundary.end_l() > -lane_width_right &&
                obj_sl_boundary.end_l() < lane_width_left)) {
      ADEBUG << " Lateral obstacle (id:" << obstacle->id()
            << ", l:" << obj_sl_boundary.start_l() << "~"
            << obj_sl_boundary.end_l() << ") is in current lane.";
      continue;
    }

    double distance_to_bound = std::numeric_limits<double>::max();
    if (NeedPayAttention(*obstacle, reference_line_info)) {
      ADEBUG << "Need pay attention to obstacle (id: " << obstacle->id() << ").";
      if (obj_sl_boundary.start_l() + obj_sl_boundary.end_l() > 0) {
        distance_to_bound = obj_sl_boundary.start_l() - lane_width_left;
        left_candidate_obstacles.insert(
            std::pair<double, std::string>(distance_to_bound, obstacle->id()));
        ADEBUG << "Distacne to left bound is " << distance_to_bound
              << " less than " << decider_config.lat_safe_buff();
      } else {
        distance_to_bound = -obj_sl_boundary.end_l() - lane_width_right;
        right_candidate_obstacles.insert(
            std::pair<double, std::string>(distance_to_bound, obstacle->id()));
        ADEBUG << "Distacne to right bound is " << distance_to_bound
              << " less than " << decider_config.lat_safe_buff();
      }
    } else {
      ADEBUG << "No need pay attention to obstacle (id: " << obstacle->id() << ").";
    }
  }
  *left_distance_to_boundary = std::get<0>(*left_candidate_obstacles.begin());
  *right_distance_to_boundary = std::get<0>(*right_candidate_obstacles.begin());
  left_target_id_ = std::get<1>(*left_candidate_obstacles.begin());
  right_target_id_ = std::get<1>(*right_candidate_obstacles.begin());
  ADEBUG << " Left target id is " << left_target_id_
        << ", distance to left boundary is " << *left_distance_to_boundary;
  ADEBUG << " Right target id is " << right_target_id_
        << ", distance to right boundary is " << *right_distance_to_boundary;
}

bool LateralShiftDecider::NeedPayAttention(
    const Obstacle& obstacle,
    const ReferenceLineInfo& reference_line_info) const {
  const auto& decider_config = config_.lateral_shift_decider_config();
  const auto& obj_sl_boundary = obstacle.sl_boundary();
  if (obstacle.is_static()) {
    bool is_in_detection_zone =
        (obj_sl_boundary.start_s() - adc_frenet_s_ <
         decider_config.forward_detection_time_length() * adc_speed_) &&
        (adc_frenet_s_ - obj_sl_boundary.end_s() < 0.0); // forward_detection_time_length:3.0
    ADEBUG << "Detection zone is ("
           << adc_frenet_s_ +
                  decider_config.forward_detection_time_length() * adc_speed_
           << " ~ " << adc_frenet_s_ << ")";
    ADEBUG << "obstacle (id:" << obstacle.id()
           << ") is static, is in detection zone? " << is_in_detection_zone;
    return is_in_detection_zone;
  }
  bool same_direction = true;
  if (obstacle.HasTrajectory()) {
    double obstacle_moving_direction =
        obstacle.trajectory().trajectory_point(0).path_point().theta();
    double vehicle_moving_direction =
        reference_line_info.vehicle_state().heading();
    double heading_difference = std::abs(pnc::NormalizeAngle(
        obstacle_moving_direction - vehicle_moving_direction));
    same_direction = heading_difference < (M_PI / 2.0);
  }

  const double ego_v = adc_speed_;
  const double obj_v = obstacle.speed();

  ADEBUG << "ego speed is " << ego_v << " , obj speed is " << obj_v;
  ADEBUG << "Obstacle is same direction with ego ? " << same_direction;
  ADEBUG << "obj_sl_boundary.start_s() " << obj_sl_boundary.start_s()
         << ", adc_frenet_s_ " << adc_frenet_s_;
  ADEBUG << "decider_config.check_time_on_same_direction() "
         << decider_config.check_time_on_same_direction(); //5s
  ADEBUG << "decider_config.check_time_on_opposite_direction() "
         << decider_config.check_time_on_opposite_direction();

  if (same_direction && obj_sl_boundary.start_s() > adc_frenet_s_) {
    // obj is ahead of ego, same direction
    if (obj_sl_boundary.start_s() - adc_frenet_s_ <
        (ego_v - obj_v) * decider_config.check_time_on_same_direction()) {
      ADEBUG << "ego caught up with obj in "
            << decider_config.check_time_on_same_direction() << " s.";
      return true;
    }
  } else if (!same_direction && obj_sl_boundary.start_s() > adc_frenet_s_) {
    // obj is ahead of ego, different direction
    if (obj_sl_boundary.start_s() - adc_frenet_s_ <
        (ego_v + obj_v) * decider_config.check_time_on_opposite_direction()) {
      ADEBUG << "ego caught up with obj in "
            << decider_config.check_time_on_opposite_direction() << " s.";
      return true;
    }
  }
  return false;
}

double LateralShiftDecider::CalculateShiftDistances(
    const double& distance_to_bound) const {
  const auto& decider_config = config_.lateral_shift_decider_config();
  double shift_distance = decider_config.lat_safe_buff() - distance_to_bound; // lat_safe_buff: 0.5
  shift_distance =
      std::min(shift_distance,
               decider_config.lateral_shift_distance_max());  // 0.3
  shift_distance = std::max(shift_distance, 0.0);
  return shift_distance;
}

void LateralShiftDecider::SelectShiftDirection(
    const double& lateral_shift_distance_left,
    const double& lateral_shift_distance_right,
    LateralShiftStatus::Type* const candidate_type,
    double* const candidate_shift_distance) {
  constexpr double kMyEpsilon = 0.01;
  target_id_ = "";
  if (lateral_shift_distance_left > kMyEpsilon &&
      lateral_shift_distance_right > kMyEpsilon) {
    ADEBUG << "left and right are both dangerous, can't shift.";
  } else if (lateral_shift_distance_left > kMyEpsilon) {
    ADEBUG << "shift to left.";
    *candidate_type = LateralShiftStatus::LEFT;
    *candidate_shift_distance = lateral_shift_distance_left;
    target_id_ = left_target_id_;
  } else if (lateral_shift_distance_right > kMyEpsilon) {
    ADEBUG << "shift to right.";
    *candidate_type = LateralShiftStatus::RIGHT;
    *candidate_shift_distance = lateral_shift_distance_right;
    target_id_ = right_target_id_;
  }
}

bool LateralShiftDecider::UpdateADCState(
    const ReferenceLineInfo& reference_line_info, const Frame& frame) {
  const pnc_map::ReferenceLine& reference_line =
      reference_line_info.reference_line();
  const pnc::TrajectoryPoint& planning_start_point =
      frame.planning_start_point();
  auto adc_sl_info =
      reference_line.GetFrenetPoint(planning_start_point.path_point());
  adc_frenet_s_ = adc_sl_info.s();
  adc_frenet_l_ = adc_sl_info.l();
  adc_speed_ = planning_start_point.v();
  ADEBUG << "adc_frenet_s is " << adc_frenet_s_ 
         << ", adc_frenet_l is " << adc_frenet_l_ 
         << ", adc_speed is " << adc_speed_;

  return true;
}

bool LateralShiftDecider::IsOutOfLane(const ReferenceLineInfo& reference_line_info) const {
  double adc_bound_left = reference_line_info.car_sl_boundary().end_l();
  double adc_bound_right = reference_line_info.car_sl_boundary().start_l();

  double curr_lane_left_width = 0.0;
  double curr_lane_right_width = 0.0;

  reference_line_info.reference_line().GetLaneWidth(
      adc_frenet_s_, curr_lane_left_width, curr_lane_right_width);

  return adc_bound_left > curr_lane_left_width || adc_bound_right < - curr_lane_right_width;
}

}  // namespace planning
}  // namespace xju
