/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/interface/ros/conversion.h"

namespace xju {
namespace pnc {

void QuaternionConversion(const common_msgs::msg::Quaternion& msg,
                          Quaternion* const proto) {
  proto->set_qx(msg.qx);
  proto->set_qy(msg.qy);
  proto->set_qz(msg.qz);
  proto->set_qw(msg.qw);
}

void LocalizationConversion(
    const localization_msgs::msg::LocalizeOutput::SharedPtr msg_ptr,
    Localization* const proto_ptr) {
  HeaderConversion(msg_ptr->header, proto_ptr->mutable_header());
  Vector3DConversion(msg_ptr->local_localize_result.position,
                     proto_ptr->mutable_position());
  Vector3DConversion(msg_ptr->local_localize_result.velocity_in_vehicle,
                     proto_ptr->mutable_linear_velocity());
  Vector3DConversion(
      msg_ptr->local_localize_result.linear_acceleration_in_vehicle,
      proto_ptr->mutable_linear_acceleration());
  Vector3DConversion(msg_ptr->local_localize_result.angular_velocity_in_vehicle,
                     proto_ptr->mutable_angular_velocity());
  QuaternionConversion(msg_ptr->local_localize_result.rotation,
                       proto_ptr->mutable_orientation());
  Vector3DConversion(msg_ptr->local_localize_result.euler_ypr,
                     proto_ptr->mutable_eular_angles());
  switch (msg_ptr->local_localize_state) {
    case localization_msgs::msg::LocalizeOutput::UNKNOWN_L:
      proto_ptr->set_state(LocalizeState::UNKNOWN_L);
      break;
    case localization_msgs::msg::LocalizeOutput::WAIT_L:
      proto_ptr->set_state(LocalizeState::WAIT_L);
      break;
    case localization_msgs::msg::LocalizeOutput::INITIALIZING_L:
      proto_ptr->set_state(LocalizeState::INITIALIZING_L);
      break;
    case localization_msgs::msg::LocalizeOutput::TRACKING_L:
      proto_ptr->set_state(LocalizeState::TRACKING_L);
      break;
    case localization_msgs::msg::LocalizeOutput::ERROR_L:
      proto_ptr->set_state(LocalizeState::ERROR_L);
      break;
    default:
      proto_ptr->set_state(LocalizeState::UNKNOWN_L);
  }
  proto_ptr->set_heading(msg_ptr->local_localize_result.euler_ypr.z);
  proto_ptr->mutable_gloal_localize()->set_x(msg_ptr->global_localize_result.global_level.mercator_x);
  proto_ptr->mutable_gloal_localize()->set_y(msg_ptr->global_localize_result.global_level.mercator_y);
  proto_ptr->mutable_gloal_localize()->set_z(msg_ptr->global_localize_result.global_level.mercator_z);
  proto_ptr->mutable_gloal_localize()->set_theta(msg_ptr->global_localize_result.global_level.yaw);
}

}  // namespace pnc
}  // namespace xju
