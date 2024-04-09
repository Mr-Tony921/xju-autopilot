/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/interface/ros/conversion.h"
#include "common/logger/logger.h"
namespace xju {
namespace pnc {

void PlanningConversion(const planning::ADCTrajectory& proto,
                        planning_msgs::msg::Planning* const msg_ptr) {
  HeaderConversion(proto.header(), &msg_ptr->header);
  msg_ptr->total_path_length = proto.total_path_length();
  msg_ptr->total_path_time = proto.total_path_time();
  msg_ptr->estop.is_estop = proto.estop().is_estop();
  msg_ptr->estop.reason = proto.estop().reason();
  msg_ptr->is_replan = proto.is_replan();
  msg_ptr->replan_reason = proto.replan_reason();

  common_msgs::msg::TrajectoryPoint point_temp;
  msg_ptr->trajectory_point.clear();
  for (const auto& trajectory_point : proto.trajectory_point()) {
    TrajectoryPointConversion(trajectory_point, &point_temp);
    msg_ptr->trajectory_point.emplace_back(point_temp);
  }

  if (proto.trajectory_type() == planning::ADCTrajectory::NORMAL) {
    msg_ptr->trajectory_type = planning_msgs::msg::Planning::NORMAL;
  } else if (proto.trajectory_type() ==
             planning::ADCTrajectory::PATH_FALLBACK) {
    msg_ptr->trajectory_type = planning_msgs::msg::Planning::PATH_FALLBACK;
  } else if (proto.trajectory_type() ==
             planning::ADCTrajectory::SPEED_FALLBACK) {
    msg_ptr->trajectory_type = planning_msgs::msg::Planning::SPEED_FALLBACK;
  } else if (proto.trajectory_type() ==
             planning::ADCTrajectory::TRAJECTORY_FALLBACK) {
    msg_ptr->trajectory_type =
        planning_msgs::msg::Planning::TRAJECTORY_FALLBACK;
  }

  msg_ptr->lane_id = proto.lane_id();
  msg_ptr->target_lane_id = proto.target_lane_id();

  if (proto.scenario_status().scenario_type() == ScenarioType::LANE_FOLLOW) {
    msg_ptr->scenario_type = planning_msgs::msg::Planning::LANE_FOLLOW;
  } else if (proto.scenario_status().scenario_type() == ScenarioType::ROAD_CHANGE) {
    msg_ptr->scenario_type = planning_msgs::msg::Planning::ROAD_CHANGE;
  } else if (proto.scenario_status().scenario_type() ==
             ScenarioType::EMERGENCY_STOP) {
    msg_ptr->scenario_type = planning_msgs::msg::Planning::EMERGENCY_STOP;
  } else if (proto.scenario_status().scenario_type() ==
             ScenarioType::EMERGENCY_PULL_OVER) {
    msg_ptr->scenario_type = planning_msgs::msg::Planning::EMERGENCY_PULL_OVER;
  } else if (proto.scenario_status().scenario_type() ==
           ScenarioType::CONTROL_LANE_FOLLOW) {
  } else if (proto.scenario_status().scenario_type() ==
             ScenarioType::CONTROL_ALIGN) {
  }

  if (proto.scenario_status().stage_type() == StageType::NO_STAGE) {
    msg_ptr->stage_type = planning_msgs::msg::Planning::NO_STAGE;
  } else if (proto.scenario_status().stage_type() ==
             StageType::LANE_FOLLOW_DEFAULT_STAGE) {
    msg_ptr->stage_type =
        planning_msgs::msg::Planning::LANE_FOLLOW_DEFAULT_STAGE;
  } else if (proto.scenario_status().stage_type() ==
             StageType::EMERGENCY_STOP_APPROACH) {
    msg_ptr->stage_type = planning_msgs::msg::Planning::EMERGENCY_STOP_APPROACH;
  } else if (proto.scenario_status().stage_type() ==
             StageType::EMERGENCY_STOP_STANDBY) {
    msg_ptr->stage_type = planning_msgs::msg::Planning::EMERGENCY_STOP_STANDBY;
  } else if (proto.scenario_status().stage_type() ==
             StageType::EMERGENCY_PULL_OVER_LANE_CHANGE) {
    msg_ptr->stage_type =
        planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_LANE_CHANGE;
  } else if (proto.scenario_status().stage_type() ==
             StageType::EMERGENCY_PULL_OVER_SLOW_DOWN) {
    msg_ptr->stage_type =
        planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_SLOW_DOWN;
  } else if (proto.scenario_status().stage_type() ==
             StageType::EMERGENCY_PULL_OVER_APPROACH) {
    msg_ptr->stage_type =
        planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_APPROACH;
  } else if (proto.scenario_status().stage_type() ==
             StageType::EMERGENCY_PULL_OVER_STANDBY) {
    msg_ptr->stage_type =
        planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_STANDBY;
  } else if (proto.scenario_status().stage_type() ==
             StageType::ROAD_CHANGE_DEFAULT_STAGE) {
    msg_ptr->stage_type =
        planning_msgs::msg::Planning::ROAD_CHANGE_DEFAULT_STAGE;
  }

  planning_msgs::msg::ObstacleDecision obstacle_decision_temp;
  msg_ptr->obstalce_decision.clear();
  for (const auto& obstacle : proto.object_decision()) {
    // TODO:skip the id conversion from proto and wait for assurance about it
    // later.
    obstacle_decision_temp.id = 0;
    if (obstacle.decision().has_ignore()) {
      obstacle_decision_temp.decision =
          planning_msgs::msg::ObstacleDecision::IGNORE;
    } else if (obstacle.decision().has_stop()) {
      obstacle_decision_temp.decision =
          planning_msgs::msg::ObstacleDecision::STOP;
    } else if (obstacle.decision().has_follow()) {
      obstacle_decision_temp.decision =
          planning_msgs::msg::ObstacleDecision::FOLLOW;
    } else if (obstacle.decision().has_yield()) {
      obstacle_decision_temp.decision =
          planning_msgs::msg::ObstacleDecision::YIELD;
    } else if (obstacle.decision().has_overtake()) {
      obstacle_decision_temp.decision =
          planning_msgs::msg::ObstacleDecision::OVERTAKE;
    } else if (obstacle.decision().has_nudge()) {
      obstacle_decision_temp.decision =
          planning_msgs::msg::ObstacleDecision::NUDGE;
    }
    msg_ptr->obstalce_decision.emplace_back(obstacle_decision_temp);
  }

  msg_ptr->lateral_shift_obstacle.id =
      proto.lateral_shift_status().obstacle_id();
  msg_ptr->lateral_shift_obstacle.direction =
      static_cast<uint8_t>(proto.lateral_shift_status().type());
  msg_ptr->lateral_shift_obstacle.distance =
      proto.lateral_shift_status().distance_m();
  msg_ptr->turn_signal = proto.turn_signal();
  msg_ptr->error_code = static_cast<uint16_t>(proto.error_code());

  msg_ptr->target_speed = proto.target_speed();

  if (proto.has_lane_change_status()) {
    if (proto.lane_change_status().status() ==
        planning::LaneChangeStatus::LANE_FOLLOW) {
      ADEBUG << "lane_follow_status in";
      ADEBUG << "LaneChangeStatus = "
             << static_cast<int>(
                    planning_msgs::msg::LaneChangeStatus::LANE_FOLLOW);
      msg_ptr->lane_change_status.status =
          planning_msgs::msg::LaneChangeStatus::LANE_FOLLOW;
    } else if (proto.lane_change_status().status() ==
               planning::LaneChangeStatus::LANE_CHANGE_PREPARE) {
      msg_ptr->lane_change_status.status =
          planning_msgs::msg::LaneChangeStatus::LANE_CHANGE_PREPARE;
    } else if (proto.lane_change_status().status() ==
               planning::LaneChangeStatus::LANE_CHANGE) {
      msg_ptr->lane_change_status.status =
          planning_msgs::msg::LaneChangeStatus::LANE_CHANGE;
    } else {
    }
    if (!proto.lane_change_status().path_id().empty()) {
      msg_ptr->lane_change_status.path_id =
        std::stoi(proto.lane_change_status().path_id());
    }
    msg_ptr->lane_change_status.timestamp =
        proto.lane_change_status().timestamp();
    msg_ptr->lane_change_status.turn_signal =
        static_cast<uint8_t>(proto.lane_change_status().turn_signal());
  } else {
  }

  if (proto.has_debug()) {
    DebugConversion(proto.debug(), &(msg_ptr->debug));
  }

  if (proto.has_latency_stats()) {
    msg_ptr->latency_stats.total_time_ms =
        proto.latency_stats().total_time_ms();
    for (const auto& task_stats : proto.latency_stats().task_stats()) {
      if (task_stats.name() == "pnc_map") {
        msg_ptr->latency_stats.pnc_map = task_stats.time_ms();
      } else if (task_stats.name() == "LATERAL_SHIFT_DECIDER") {
        msg_ptr->latency_stats.lateral_shift_decider = task_stats.time_ms();
      } else if (task_stats.name() == "PATH_BOUND_DECIDER") {
        msg_ptr->latency_stats.path_bound_decider = task_stats.time_ms();
      } else if (task_stats.name() == "PATH_OPTIMIZER") {
        msg_ptr->latency_stats.path_optimizer = task_stats.time_ms();
      } else if (task_stats.name() == "PATH_DECIDER") {
        msg_ptr->latency_stats.path_decider = task_stats.time_ms();
      } else if (task_stats.name() == "ST_GRAPH_DECIDER") {
        msg_ptr->latency_stats.st_graph_decider = task_stats.time_ms();
      } else if (task_stats.name() == "SPEED_DECIDER") {
        msg_ptr->latency_stats.speed_decider = task_stats.time_ms();
      } else if (task_stats.name() == "SPEED_OPTIMIZER") {
        msg_ptr->latency_stats.speed_optimizer = task_stats.time_ms();
      }
    }
  }

  if (proto.has_localize_pose()) {
    msg_ptr->localize_state.x = proto.localize_pose().x();
    msg_ptr->localize_state.y = proto.localize_pose().y();
    msg_ptr->localize_state.theta = proto.localize_pose().theta();
  }

  if (proto.has_estop()) {
    msg_ptr->estop.is_estop = proto.estop().is_estop();
    msg_ptr->estop.reason = proto.estop().reason();
  } else {
    msg_ptr->estop.is_estop = false;
    msg_ptr->estop.reason = "";
  }
}

// msg to proto
void PlanningConversion(const planning_msgs::msg::Planning::SharedPtr msg_ptr,
                        planning::ADCTrajectory* const proto) {
  HeaderConversion(msg_ptr->header, proto->mutable_header());

  proto->set_total_path_length(msg_ptr->total_path_length);
  proto->set_total_path_time(msg_ptr->total_path_time);
  proto->mutable_estop()->set_is_estop(msg_ptr->estop.is_estop);
  proto->mutable_estop()->set_reason(msg_ptr->estop.reason);
  proto->set_is_replan(msg_ptr->is_replan);
  proto->set_replan_reason(msg_ptr->replan_reason);

  pnc::TrajectoryPoint point_temp;
  std::vector<pnc::TrajectoryPoint> point_temp_vec;
  for (const auto& trajectory_point : msg_ptr->trajectory_point) {
    TrajectoryPointConversion(trajectory_point, &point_temp);
    point_temp_vec.emplace_back(point_temp);
  }
  proto->mutable_trajectory_point()->CopyFrom(
      {point_temp_vec.begin(), point_temp_vec.end()});

  if (msg_ptr->trajectory_type == planning_msgs::msg::Planning::NORMAL) {
    proto->set_trajectory_type(planning::ADCTrajectory::NORMAL);
  } else if (msg_ptr->trajectory_type ==
             planning_msgs::msg::Planning::PATH_FALLBACK) {
    proto->set_trajectory_type(planning::ADCTrajectory::PATH_FALLBACK);
  } else if (msg_ptr->trajectory_type ==
             planning::ADCTrajectory::SPEED_FALLBACK) {
    proto->set_trajectory_type(planning::ADCTrajectory::SPEED_FALLBACK);
  } else if (msg_ptr->trajectory_type ==
             planning_msgs::msg::Planning::TRAJECTORY_FALLBACK) {
    proto->set_trajectory_type(planning::ADCTrajectory::TRAJECTORY_FALLBACK);
  }

  proto->set_lane_id(msg_ptr->lane_id);
  proto->set_target_lane_id(msg_ptr->target_lane_id);

  if (msg_ptr->scenario_type == planning_msgs::msg::Planning::LANE_FOLLOW) {
    proto->mutable_scenario_status()->set_scenario_type(
        ScenarioType::LANE_FOLLOW);
  } else if (msg_ptr->scenario_type ==
             planning_msgs::msg::Planning::EMERGENCY_STOP) {
    proto->mutable_scenario_status()->set_scenario_type(
        ScenarioType::EMERGENCY_STOP);
  } else if (msg_ptr->scenario_type ==
             planning_msgs::msg::Planning::EMERGENCY_PULL_OVER) {
    proto->mutable_scenario_status()->set_scenario_type(
        ScenarioType::EMERGENCY_PULL_OVER);
  } else if (msg_ptr->scenario_type == planning_msgs::msg::Planning::ROAD_CHANGE) {
    proto->mutable_scenario_status()->set_scenario_type(
        ScenarioType::ROAD_CHANGE);
  }

  if (msg_ptr->stage_type == planning_msgs::msg::Planning::NO_STAGE) {
    proto->mutable_scenario_status()->set_stage_type(StageType::NO_STAGE);
  } else if (msg_ptr->stage_type ==
             planning_msgs::msg::Planning::LANE_FOLLOW_DEFAULT_STAGE) {
    proto->mutable_scenario_status()->set_stage_type(
        StageType::LANE_FOLLOW_DEFAULT_STAGE);
  } else if (msg_ptr->stage_type ==
             planning_msgs::msg::Planning::ROAD_CHANGE_DEFAULT_STAGE) {
    proto->mutable_scenario_status()->set_stage_type(
        StageType::ROAD_CHANGE_DEFAULT_STAGE);
  } else if (msg_ptr->stage_type ==
             planning_msgs::msg::Planning::EMERGENCY_STOP_APPROACH) {
    proto->mutable_scenario_status()->set_stage_type(
        StageType::EMERGENCY_STOP_APPROACH);
  } else if (msg_ptr->stage_type ==
             planning_msgs::msg::Planning::EMERGENCY_STOP_STANDBY) {
    proto->mutable_scenario_status()->set_stage_type(
        StageType::EMERGENCY_STOP_STANDBY);
  } else if (msg_ptr->stage_type ==
             planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_LANE_CHANGE) {
    proto->mutable_scenario_status()->set_stage_type(
        StageType::EMERGENCY_PULL_OVER_LANE_CHANGE);
  } else if (msg_ptr->stage_type ==
             planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_SLOW_DOWN) {
    proto->mutable_scenario_status()->set_stage_type(
        StageType::EMERGENCY_PULL_OVER_SLOW_DOWN);
  } else if (msg_ptr->stage_type ==
             planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_APPROACH) {
    proto->mutable_scenario_status()->set_stage_type(
        StageType::EMERGENCY_PULL_OVER_APPROACH);
  } else if (msg_ptr->stage_type ==
             planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_STANDBY) {
    proto->mutable_scenario_status()->set_stage_type(
        StageType::EMERGENCY_PULL_OVER_STANDBY);
  }

  planning::ObjectDecision obstacle_decision_temp;
  std::vector<planning::ObjectDecision> obstacle_decision_temp_vec;

  for (const auto& obstacle : msg_ptr->obstalce_decision) {
    // TODO:skip the id conversion from proto and wait for assurance about it
    // later.
    obstacle_decision_temp.set_id(0);
    if (obstacle.decision == planning_msgs::msg::ObstacleDecision::IGNORE) {
      planning::ObjectIgnore ignore;
      obstacle_decision_temp.mutable_decision()->set_allocated_ignore(&ignore);
    } else if (obstacle.decision ==
               planning_msgs::msg::ObstacleDecision::STOP) {
      planning::ObjectStop stop;
      obstacle_decision_temp.mutable_decision()->set_allocated_stop(&stop);
    } else if (obstacle.decision ==
               planning_msgs::msg::ObstacleDecision::FOLLOW) {
      planning::ObjectFollow follow;
      obstacle_decision_temp.mutable_decision()->set_allocated_follow(&follow);
    } else if (obstacle.decision ==
               planning_msgs::msg::ObstacleDecision::YIELD) {
      planning::ObjectYield yield;
      obstacle_decision_temp.mutable_decision()->set_allocated_yield(&yield);
    } else if (obstacle.decision ==
               planning_msgs::msg::ObstacleDecision::OVERTAKE) {
      planning::ObjectOvertake overtake;
      obstacle_decision_temp.mutable_decision()->set_allocated_overtake(
          &overtake);
    } else if (obstacle.decision ==
               planning_msgs::msg::ObstacleDecision::NUDGE) {
      planning::ObjectNudge nudge;
      obstacle_decision_temp.mutable_decision()->set_allocated_nudge(&nudge);
    }
    obstacle_decision_temp_vec.emplace_back(obstacle_decision_temp);
  }
  proto->mutable_object_decision()->CopyFrom(
      {obstacle_decision_temp_vec.begin(), obstacle_decision_temp_vec.end()});

  proto->mutable_lateral_shift_status()->set_obstacle_id(
      msg_ptr->lateral_shift_obstacle.id);

  switch (msg_ptr->lateral_shift_obstacle.direction) {
    case 1:
      proto->mutable_lateral_shift_status()->set_type(
          planning::LateralShiftStatus::LEFT);
      break;
    case 2:
      proto->mutable_lateral_shift_status()->set_type(
          planning::LateralShiftStatus::RIGHT);
      break;
    default:
      proto->mutable_lateral_shift_status()->set_type(
          planning::LateralShiftStatus::NONE);
      break;
  }

  proto->mutable_lateral_shift_status()->set_distance_m(
      msg_ptr->lateral_shift_obstacle.distance);

  switch (msg_ptr->turn_signal) {
    case 1:
      proto->set_turn_signal(TurnSignal::TURN_LEFT);
      break;
    case 2:
      proto->set_turn_signal(TurnSignal::TURN_RIGHT);
      break;
    default:
      proto->set_turn_signal(TurnSignal::TURN_NONE);
      break;
  }

  switch (msg_ptr->error_code) {
    case 6000:
      proto->set_error_code(ErrorCode::PLANNING_ERROR);
      break;
    case 6001:
      proto->set_error_code(ErrorCode::PLANNING_ERROR_NOT_READY);
      break;
    default:
      proto->set_error_code(ErrorCode::OK);
      break;
  }

  proto->set_target_speed(msg_ptr->target_speed);

  if (msg_ptr->lane_change_status.status ==
      planning_msgs::msg::LaneChangeStatus::LANE_FOLLOW) {
    proto->mutable_lane_change_status()->set_status(
        planning::LaneChangeStatus::LANE_FOLLOW);
  } else if (msg_ptr->lane_change_status.status ==
             planning_msgs::msg::LaneChangeStatus::LANE_CHANGE_PREPARE) {
    proto->mutable_lane_change_status()->set_status(
        planning::LaneChangeStatus::LANE_CHANGE_PREPARE);
  } else if (msg_ptr->lane_change_status.status =
                 planning_msgs::msg::LaneChangeStatus::LANE_CHANGE) {
    proto->mutable_lane_change_status()->set_status(
        planning::LaneChangeStatus::LANE_CHANGE);
  } else {
  }
  std::string path_id = std::to_string(msg_ptr->lane_change_status.path_id);
  proto->mutable_lane_change_status()->set_path_id(path_id);

  proto->mutable_lane_change_status()->set_timestamp(
      msg_ptr->lane_change_status.timestamp);

  switch (msg_ptr->lane_change_status.turn_signal) {
    case 1:
      proto->mutable_lane_change_status()->set_turn_signal(
          TurnSignal::TURN_LEFT);
      break;
    case 2:
      proto->mutable_lane_change_status()->set_turn_signal(
          TurnSignal::TURN_RIGHT);
      break;
    default:
      proto->mutable_lane_change_status()->set_turn_signal(
          TurnSignal::TURN_NONE);
      break;
  }

  auto* localize_pose = proto->mutable_localize_pose();
  localize_pose->set_x(msg_ptr->localize_state.x);
  localize_pose->set_y(msg_ptr->localize_state.y);
  localize_pose->set_theta(msg_ptr->localize_state.theta);

  proto->mutable_latency_stats()->set_total_time_ms(
      msg_ptr->latency_stats.total_time_ms);
  // if (proto.has_debug()) {
  //   DebugConversion(proto.debug(), &(msg_ptr->debug));
  // }
  proto->mutable_estop()->set_is_estop(msg_ptr->estop.is_estop);
  proto->mutable_estop()->set_reason(msg_ptr->estop.reason);
}

}  // namespace pnc
}  // namespace xju
