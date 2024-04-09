/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/vehicle_state/vehicle_state_provider.h"

#include <cmath>

#include "common/logger/logger.h"
#include "common/math/euler_angles_zxy.h"
#include "common/math/math_utils.h"
#include "common/time/time.h"
#include "common/math/math_utils.h"

namespace xju {
namespace pnc {

VehicleState VehicleStateProvider::vehicle_state_;
VehicleState VehicleStateProvider::last_vehicle_state_;
VehicleStateConfig VehicleStateProvider::vehicle_state_config_;
VehicleConfig VehicleStateProvider::vehicle_config_;
double VehicleStateProvider::p_ = 0.0;
MeanFilter VehicleStateProvider::mean_filter_;
double VehicleStateProvider::go_straight_distance_ = 0.0;
bool VehicleStateProvider::is_first_compute_ = true;
LocalizePose VehicleStateProvider::localize_pose_;
LocalizePose VehicleStateProvider::last_localize_pose_;

void VehicleStateProvider::Init(const VehicleStateConfig& config) {
  // AINFO << config.ShortDebugString();
  vehicle_config_ = VehicleConfigProvider::GetConfig();
  vehicle_state_config_ = config;
  mean_filter_ = MeanFilter(vehicle_state_config_.mean_filter_window());
  go_straight_distance_ = 0.0;
  is_first_compute_ = true;
}

void VehicleStateProvider::Clear() {
  mean_filter_.Clear();
  is_first_compute_ = true;
  go_straight_distance_ = 0.0;
  p_ = 0.0;
}

bool VehicleStateProvider::Update(
    const Localization& localization,
    const ChassisInfo& chassis_info) {

  if (!ConstructFromLocalization(localization, chassis_info)) {
    Clear();
    return false;
  }

  if (!ConstructFromChassis(chassis_info)) {
    Clear();
    return false;
  }

  if (is_first_compute_) {
    last_vehicle_state_ = vehicle_state_;
    last_localize_pose_ = localize_pose_;
    is_first_compute_ = false;
  }

  bool go_straight = IsCarGoStraight();
  
  last_vehicle_state_ = vehicle_state_;
  last_localize_pose_ = localize_pose_;
  return true;
}

const VehicleState& VehicleStateProvider::GetVehicleState() {
  return vehicle_state_;
}

const LocalizePose VehicleStateProvider::localize_pose() {
  return localize_pose_;
}
  
const double VehicleStateProvider::car_x() {
  return vehicle_state_.x();
}

const double VehicleStateProvider::car_y() {
  return vehicle_state_.y();
}

const double VehicleStateProvider::car_heading() {
  return vehicle_state_.heading();
}

const double VehicleStateProvider::roll() {
  return vehicle_state_.roll();
}

const double VehicleStateProvider::pitch() {
  return vehicle_state_.pitch();
}

const double VehicleStateProvider::yaw() {
  return vehicle_state_.yaw();
}

const double VehicleStateProvider::kappa() {
  return vehicle_state_.kappa();
}

const double VehicleStateProvider::speed() {
  return vehicle_state_.linear_velocity();
}

const double VehicleStateProvider::front_wheel_angle() {
  return vehicle_state_.front_wheel_angle();
}

const double VehicleStateProvider::steering_angle() {
  return vehicle_state_.steering_angle();
}

const double VehicleStateProvider::acceleration() {
  return vehicle_state_.linear_acceleration();
}

const double VehicleStateProvider::mass() {
  return vehicle_state_.mass();
}

const double VehicleStateProvider::timestamp() {
  return vehicle_state_.timestamp();
}

bool VehicleStateProvider::ConstructFromLocalization(
    const Localization& localization, const ChassisInfo& chassis_info) {
  if (!localization.has_position()) {
    AERROR << "Invalid localization input.";
    return false;
  }

  vehicle_state_.set_x(0.0);
  vehicle_state_.set_y(0.0);
  vehicle_state_.set_z(0.0);
  vehicle_state_.set_heading(0.0);

  localize_pose_.x = localization.position().x();
  localize_pose_.y = localization.position().y();
  
  if (localization.has_header()) {
    vehicle_state_.set_timestamp(localization.header().timestamp_sec());
  } else {
    AERROR << "Invalid localization input. localization header is empty.";
    return false;
  }

  if (!localization.has_orientation()) {
    AERROR << "Invalid localization input. localization orientation is empty.";
    return false;
  }
  const auto& orientation = localization.orientation();
  if (localization.has_heading()) {
    localize_pose_.heading = localization.heading();
  } else {
    localize_pose_.heading = QuaternionToHeading(orientation.qw(), orientation.qx(),
                                                 orientation.qy(), orientation.qz());
  }
  localize_pose_.is_valid = true;
  
  const double rear_axle_to_center = 
      vehicle_config_.rear_axle_to_center();
  vehicle_state_.set_center_x(
      car_x() + rear_axle_to_center * std::cos(car_heading()));
  vehicle_state_.set_center_y(
      car_y() + rear_axle_to_center * std::sin(car_heading()));

  if (localization.has_linear_velocity()) {
    auto speed_value = std::abs(localization.linear_velocity().x()) < 0.1
                           ? 0.0
                           : localization.linear_velocity().x();
    vehicle_state_.set_linear_velocity(speed_value);
  } else {
    AERROR << "Invalid localization input. localization linear_velocity is empty.";
    // return false;
    if (!chassis_info.has_car_speed_info()) {
      AERROR << "Invalid chassis input. chassis car_speed_info is empty.";
      return false;
    }
    const auto& car_speed_info = chassis_info.car_speed_info();
    if (car_speed_info.has_speed()) {
      vehicle_state_.set_linear_velocity(car_speed_info.speed());
    } else {
      AERROR << "Invalid car_speed_info input. car_speed_info speed_ms is empty.";
      return false;
    }
  }

  if (localization.has_linear_acceleration()) {
    if (localization.linear_acceleration().x() >= 0.05) {
      vehicle_state_.set_linear_acceleration(
          localization.linear_acceleration().x());
    } else {
      vehicle_state_.set_linear_acceleration(0.0);
    }
  } else {
    AERROR << "Invalid localization input. localization linear_acceleration is empty.";
    if (!chassis_info.has_car_speed_info()) {
      AERROR << "Invalid chassis input. chassis car_speed_info is empty.";
      return false;
    }
    if (chassis_info.car_speed_info().has_acceleration()) {
      vehicle_state_.set_linear_acceleration(
          chassis_info.car_speed_info().acceleration());
    } else {
      AERROR << "Invalid car_speed_info input. car_speed_info acceleration is empty.";
      return false;
    }
  }

  if (localization.has_angular_velocity()) {
    vehicle_state_.set_angular_velocity(localization.angular_velocity().z());
  } else {
    AERROR << "Invalid localization input. localization angular_velocity is empty.";
    return false;
  }

  if (localization.has_eular_angles()) {
    vehicle_state_.set_roll(localization.eular_angles().y());
    vehicle_state_.set_pitch(localization.eular_angles().x());
    vehicle_state_.set_yaw(localization.eular_angles().z());
  } else {
    AERROR << "Invalid localization input. localization eular_angles is empty.";
    EulerAnglesZXYd euler_angle(orientation.qw(), orientation.qx(),
                                orientation.qy(), orientation.qz());
    vehicle_state_.set_roll(euler_angle.roll());
    vehicle_state_.set_pitch(euler_angle.pitch());
    vehicle_state_.set_yaw(euler_angle.yaw());
  }

  return true;
}

bool VehicleStateProvider::ConstructFromChassis(const ChassisInfo& chassis_info) {
  if (chassis_info.has_gear_info() && 
      chassis_info.gear_info().has_gearbox_actual()) {
    vehicle_state_.set_gear(chassis_info.gear_info().gearbox_actual());
  } else {
    AERROR << "Invalid chassis input. chassis gear_position is empty.";
    vehicle_state_.set_gear(GearPosition::NONE);
    return false;
  }

  if (chassis_info.has_steer_info() && 
      chassis_info.steer_info().has_steer_angle_actual()) {
    vehicle_state_.set_steering_angle(
        chassis_info.steer_info().steer_angle_actual());
    double steer_ratio = vehicle_config_.steer_ratio();
    vehicle_state_.set_front_wheel_angle(ToRadian(steering_angle() / steer_ratio));
  } else {
    AERROR << "Invalid chassis input. chassis steering_angle is empty.";
    return false;
  }

  if (std::fabs(vehicle_state_.linear_velocity()) < kMathEpsilon) {
    vehicle_state_.set_kappa(0.0);
  } else {
    vehicle_state_.set_kappa(
        vehicle_state_.angular_velocity() / vehicle_state_.linear_velocity());
  }

  if (chassis_info.has_car_mass_info() &&
      chassis_info.car_mass_info().has_vehicle_mass()) {
    vehicle_state_.set_mass(chassis_info.car_mass_info().vehicle_mass());
  } else {
    AERROR << "Invalid chassis input. chassis mass is empty.";
    return false;
  }
  return true;
}

bool VehicleStateProvider::IsCarGoStraight() {
  double last_speed = last_vehicle_state_.linear_velocity();
  double last_timestamp = last_vehicle_state_.timestamp();
  double distance = (speed() + last_speed) * (timestamp() - last_timestamp) / 2.0;
  if (speed() > 0.1 && std::fabs(kappa()) < 0.01) {
    go_straight_distance_ += distance;
  } else {
    go_straight_distance_ -= distance;
  }
  go_straight_distance_ = (go_straight_distance_ > kMathEpsilon)
                              ? go_straight_distance_ : 0;
  go_straight_distance_ = std::fmin(go_straight_distance_, 
                                    vehicle_state_config_.go_straight_distance() + 20.0);
  return go_straight_distance_ >= vehicle_state_config_.go_straight_distance();
}

} // namespace pnc
} // namespace xju
