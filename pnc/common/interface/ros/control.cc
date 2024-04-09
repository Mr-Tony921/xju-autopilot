/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/interface/ros/conversion.h"
#include "common/logger/logger.h"

namespace xju {
namespace pnc {

// msg 2 proto
void ControlConversion(const control_msgs::msg::Control::SharedPtr msg,
                       control::ControlCommand* const proto) {
  HeaderConversion(msg->header, proto->mutable_header());
  proto->set_wheel_torque(msg->wheel_torque);
  proto->set_decelerate(msg->decelerate);
  proto->set_steering(msg->steering);
  proto->set_acceleration(msg->acceleration);
  proto->set_speed(msg->speed);
  proto->set_steering_rate(msg->steering_rate);
  proto->set_parking_brake(msg->parking_brake);
  proto->set_emergency_light(msg->emergency_light);
  proto->set_vehicle_mass(msg->vehicle_mass);

  proto->set_wheel_torque_isvalid(msg->wheel_torque_isvalid);
  proto->set_decelerate_isvalid(msg->decelerate_isvalid);
  proto->set_steering_isvalid(msg->steering_isvalid);

  // gear
  switch (msg->gear_position) {
    case 0:
      proto->set_gear(GearPosition::NEUTRAL);
      break;
    case 1:
      proto->set_gear(GearPosition::DRIVE);
      break;
    case 2:
      proto->set_gear(GearPosition::REVERSE);
      break;
    case 3:
      proto->set_gear(GearPosition::PARKING);
      break;
    case 4:
      proto->set_gear(GearPosition::LOW);
      break;
    case 5:
      proto->set_gear(GearPosition::INVALID);
      break;
    default:
      proto->set_gear(GearPosition::NONE);
      break;
  }

  switch (msg->turn_signal) {
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

  // debug info, error code
  switch (msg->error_code) {
    case 1000:
      proto->set_error_code(ErrorCode::CONTROL_ERROR);
      break;
    case 1001:
      proto->set_error_code(ErrorCode::CONTROL_INIT_ERROR);
      break;
    case 1002:
      proto->set_error_code(ErrorCode::CONTROL_COMPUTE_ERROR);
      break;
    default:
      proto->set_error_code(ErrorCode::OK);
      break;
  }
}

// proto 2 msg
void ControlConversion(const control::ControlCommand& proto,
                       control_msgs::msg::Control* const msg) {
  HeaderConversion(proto.header(), &(msg->header));
  msg->wheel_torque = proto.wheel_torque();
  msg->decelerate = proto.decelerate();
  msg->steering = proto.steering();
  msg->acceleration = proto.acceleration();
  msg->speed = proto.speed();
  msg->steering_rate = proto.steering_rate();
  msg->parking_brake = proto.parking_brake();
  msg->emergency_light = proto.emergency_light();
  msg->vehicle_mass = proto.vehicle_mass();

  msg->wheel_torque_isvalid = proto.wheel_torque_isvalid();
  msg->decelerate_isvalid = proto.decelerate_isvalid();
  msg->steering_isvalid = proto.steering_isvalid();

  // gear
  switch (proto.gear()) {
    case GearPosition::NEUTRAL:
      msg->gear_position = 0;
      break;
    case GearPosition::DRIVE:
      msg->gear_position = 1;
      break;
    case GearPosition::REVERSE:
      msg->gear_position = 2;
      break;
    case GearPosition::PARKING:
      msg->gear_position = 3;
      break;
    case GearPosition::LOW:
      msg->gear_position = 4;
      break;
    case GearPosition::INVALID:
      msg->gear_position = 5;
      break;
    default:
      msg->gear_position = 6;
      break;
  }

  switch (proto.turn_signal()) {
    case TurnSignal::TURN_LEFT:
      msg->turn_signal = 1;
      break;
    case TurnSignal::TURN_RIGHT:
      msg->turn_signal = 2;
      break;
    default:
      msg->turn_signal = 0;
      break;
  }

  if (proto.has_visualization_debug()) {
    DebugConversion(proto.visualization_debug(), &(msg->visualization_debug));
  }

  // debug info
  msg->debug.cost_time = proto.debug().cost_time();
  msg->debug.record_time = proto.debug().record_time();

  for (const auto trajectory :
       proto.debug().mpc_debug().resample_trajectory()) {
    common_msgs::msg::TrajectoryPoint point;
    TrajectoryPointConversion(trajectory, &point);
    msg->debug.mpc_debug.mpc_resample_trajectory.emplace_back(point);
  }

  for (const auto trajectory :
       proto.debug().mpc_debug().predicted_trajectory()) {
    common_msgs::msg::TrajectoryPoint point;
    TrajectoryPointConversion(trajectory, &point);
    msg->debug.mpc_debug.mpc_predicted_trajectory.emplace_back(point);
  }

  TrajectoryPointConversion(proto.debug().mpc_debug().matched_point(),
                            &(msg->debug.mpc_debug.matched_point));

  TrajectoryPointConversion(proto.debug().longitudinal_debug().preview_point(),
                            &(msg->debug.longitudinal_debug.preview_point));

  for (const auto state : proto.debug().mpc_debug().matrix_q_updated()) {
    msg->debug.mpc_debug.matrix_q_updated.emplace_back(state);
  }

  for (const auto state : proto.debug().mpc_debug().matrix_r_updated()) {
    msg->debug.mpc_debug.matrix_r_updated.emplace_back(state);
  }

  for (const auto state : proto.debug().mpc_debug().matrix_state()) {
    msg->debug.mpc_debug.matrix_state.emplace_back(state);
  }

  msg->debug.mpc_debug.calculated_cost_time =
      proto.debug().mpc_debug().calculated_cost_time();
  msg->debug.mpc_debug.acceleration = proto.debug().mpc_debug().acceleration();
  msg->debug.mpc_debug.steering = proto.debug().mpc_debug().steering();
  msg->debug.mpc_debug.jerk = proto.debug().mpc_debug().jerk();
  msg->debug.mpc_debug.steer_angle_rate =
      proto.debug().mpc_debug().steer_angle_rate();
  msg->debug.mpc_debug.mpc_x_error_average =
      proto.debug().mpc_debug().mpc_x_error_average();
  msg->debug.mpc_debug.mpc_y_error_average =
      proto.debug().mpc_debug().mpc_y_error_average();
  msg->debug.mpc_debug.mpc_v_error_average =
      proto.debug().mpc_debug().mpc_v_error_average();
  msg->debug.mpc_debug.mpc_phi_error_average =
      proto.debug().mpc_debug().mpc_phi_error_average();
  msg->debug.mpc_debug.iteration_number =
      proto.debug().mpc_debug().iteration_number();
  msg->debug.mpc_debug.solve_status = proto.debug().mpc_debug().solve_status();
  msg->debug.mpc_debug.cost_time = proto.debug().mpc_debug().cost_time();
  msg->debug.mpc_debug.state_cost_precept =
      proto.debug().mpc_debug().state_cost_precept();
  msg->debug.mpc_debug.control_cost_precept =
      proto.debug().mpc_debug().control_cost_precept();
  msg->debug.mpc_debug.control_rate_cost_precept =
      proto.debug().mpc_debug().control_rate_cost_precept();
  msg->debug.mpc_debug.cost_of_state_x =
      proto.debug().mpc_debug().cost_of_state_x();
  msg->debug.mpc_debug.cost_of_state_y =
      proto.debug().mpc_debug().cost_of_state_y();
  msg->debug.mpc_debug.cost_of_state_v =
      proto.debug().mpc_debug().cost_of_state_v();
  msg->debug.mpc_debug.cost_of_state_phi =
      proto.debug().mpc_debug().cost_of_state_phi();

  msg->debug.longitudinal_debug.is_full_stop =
      proto.debug().longitudinal_debug().is_full_stop();
  msg->debug.longitudinal_debug.path_remain =
      proto.debug().longitudinal_debug().path_remain();
  msg->debug.longitudinal_debug.vehicle_mass =
      proto.debug().longitudinal_debug().vehicle_mass();
  msg->debug.longitudinal_debug.cost_time =
      proto.debug().longitudinal_debug().cost_time();

  msg->debug.lateral_debug.real_steer_wheel_angle =
      proto.debug().lateral_debug().real_steer_wheel_angle();
  msg->debug.lateral_debug.cost_time =
      proto.debug().lateral_debug().cost_time();

  msg->debug.pre_debug.localization_x =
      proto.debug().pre_debug().localization_x();
  msg->debug.pre_debug.localization_y =
      proto.debug().pre_debug().localization_y();
  msg->debug.pre_debug.localization_heading =
      proto.debug().pre_debug().localization_heading();
  msg->debug.pre_debug.cost_time = proto.debug().pre_debug().cost_time();
  msg->debug.pre_debug.test_1 = proto.debug().pre_debug().test_1();

  msg->debug.post_debug.test = proto.debug().post_debug().test();
  msg->debug.post_debug.cost_time = proto.debug().post_debug().cost_time();

  msg->debug.mass_identification_debug.test =
      proto.debug().mass_identification_debug().test();
  msg->debug.mass_identification_debug.cost_time =
      proto.debug().mass_identification_debug().cost_time();

  // error code
  switch (proto.error_code()) {
    case ErrorCode::CONTROL_ERROR:
      msg->error_code = 1000;
      break;
    case ErrorCode::CONTROL_INIT_ERROR:
      msg->error_code = 1001;
      break;
    case ErrorCode::CONTROL_COMPUTE_ERROR:
      msg->error_code = 1002;
      break;
    default:
      msg->error_code = 0;
      break;
  }
}

}  // namespace pnc
}  // namespace xju