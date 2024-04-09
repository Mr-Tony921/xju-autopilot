/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cmath>

#include "common/math/vec2d.h"
#include "common/math/box2d.h"
#include "pnc_point.pb.h"
#include "common/vehicle_config/vehicle_config_provider.h"

namespace xju {
namespace pnc {

class VehicleHelper {
 public:
  VehicleHelper() = delete;
  ~VehicleHelper() = default;

  static Vec2d CarCenter(const PathPoint& pt) {
    const VehicleConfig& vehicle_config = VehicleConfigProvider::GetConfig();
    const double rear_axle_to_center = vehicle_config.rear_axle_to_center();
    double x = pt.x() + rear_axle_to_center * std::cos(pt.theta());
    double y = pt.y() + rear_axle_to_center * std::sin(pt.theta());
    return {x, y};
  }

  static Box2d CarBox(const PathPoint& pt) {
    const VehicleConfig& vehicle_config = VehicleConfigProvider::GetConfig();
    const double rear_axle_to_center = vehicle_config.rear_axle_to_center();
    static const double car_length = vehicle_config.length();
    static const double car_width = vehicle_config.width();
    double x = pt.x() + rear_axle_to_center * std::cos(pt.theta());
    double y = pt.y() + rear_axle_to_center * std::sin(pt.theta());
    return Box2d(x, y, pt.theta(), car_length, car_width);
  }

  static bool IsVehicleStateValid(const VehicleState& vehicle_state) {
    if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
        std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
        std::isnan(vehicle_state.kappa()) ||
        std::isnan(vehicle_state.linear_velocity()) ||
        std::isnan(vehicle_state.linear_acceleration()) ||
        std::isnan(vehicle_state.center_x()) ||
        std::isnan(vehicle_state.center_y())) {
      return false;
    }

    return true;
  }

};

} //namespace pnc
} //namespace xju
