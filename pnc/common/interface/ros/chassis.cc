/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/interface/ros/conversion.h"

namespace xju {
namespace pnc {

void BrakeInfoConversion(const chassis_msgs::msg::BrakeInfo::SharedPtr msg_ptr,
                         BrakeInfo* const proto_ptr) {
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());
  proto_ptr->set_deceleration_command(msg_ptr->deceleration_command);
  proto_ptr->set_deceleration_actual(msg_ptr->deceleration_actual);
  proto_ptr->set_pedal_command(msg_ptr->pedal_command);
  proto_ptr->set_pedal_actual(msg_ptr->pedal_actual);
  proto_ptr->set_ebs_override(msg_ptr->ebs_override);
  proto_ptr->set_epb_override(msg_ptr->epb_override);
  proto_ptr->set_ebs_fault(msg_ptr->ebs_fault);
  proto_ptr->set_epb_fault(msg_ptr->epb_fault);
  proto_ptr->set_epb_status(
      static_cast<BrakeInfo_EpbStatus>(msg_ptr->epb_status));
}

void ThrottleInfoConversion(
    const chassis_msgs::msg::ThrottleInfo::SharedPtr msg_ptr,
    ThrottleInfo* const proto_ptr) {
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());
  proto_ptr->set_pedal_output(msg_ptr->pedal_output);
  proto_ptr->set_pedal_command(msg_ptr->pedal_command);
  proto_ptr->set_motor_torque_vaild(msg_ptr->motor_torque_vaild);
  proto_ptr->set_motor_torque_command(msg_ptr->motor_torque_command);
  proto_ptr->set_motor_torque_actual(msg_ptr->motor_torque_actual);
  proto_ptr->set_throttle_override(msg_ptr->throttle_override);
  proto_ptr->set_throttle_fault(msg_ptr->throttle_fault);
  proto_ptr->set_motor_speed(msg_ptr->motor_speed);
  proto_ptr->set_veh_max_drv_tq(msg_ptr->veh_max_drv_tq);
  // proto_ptr->set_veh_max_rgn_tq(msg_ptr->veh_max_rgn_tq);
}

void SteerInfoConversion(const chassis_msgs::msg::SteerInfo::SharedPtr msg_ptr,
                         SteerInfo* const proto_ptr) {
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());
  proto_ptr->set_steer_angle_isvalid(msg_ptr->steer_angle_isvalid);
  proto_ptr->set_steer_angle_command(msg_ptr->steer_angle_command);
  proto_ptr->set_steer_angle_actual(msg_ptr->steer_angle_actual);
  proto_ptr->set_steer_angle_speed_isvalid(msg_ptr->steer_angle_speed_isvalid);
  proto_ptr->set_steer_angle_speed(msg_ptr->steer_angle_speed);
  proto_ptr->set_steer_torque(msg_ptr->steer_torque);
  proto_ptr->set_steer_override(msg_ptr->steer_override);
  proto_ptr->set_steer_fault(msg_ptr->steer_fault);
}

void CarSpeedInfoConversion(
    const chassis_msgs::msg::CarSpeedInfo::SharedPtr msg_ptr,
    CarSpeedInfo* const proto_ptr) {
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());
  proto_ptr->set_isvalid(msg_ptr->isvalid);
  proto_ptr->set_speed(msg_ptr->speed);
  proto_ptr->set_acceleration(msg_ptr->acceleration);
  proto_ptr->set_direction(static_cast<Direction>(msg_ptr->direction));
}

void CarMassInfoConversion(
    const chassis_msgs::msg::CarMassInfo::SharedPtr msg_ptr,
    CarMassInfo* const proto_ptr) {
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());
  proto_ptr->set_vehicle_mass(msg_ptr->vehicle_mass);
}

void GearBoxInfoConversion(
    const chassis_msgs::msg::GearBoxInfo::SharedPtr msg_ptr,
    GearBoxInfo* const proto_ptr) {
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());
  proto_ptr->set_gearbox_command(
      static_cast<GearPosition>(msg_ptr->gearbox_command));
  proto_ptr->set_gearbox_actual(
      static_cast<GearPosition>(msg_ptr->gearbox_actual));
  proto_ptr->set_gearbox_fault(msg_ptr->gearbox_fault);
}

}  // namespace pnc
}  // namespace xju