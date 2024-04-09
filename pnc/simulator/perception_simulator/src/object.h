/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace xju {
namespace simulator {

enum class ObjectClass {
  UNKNOW = 0,
  CAR = 1,
  TRUCK = 2,
  PEOPLE = 3,
  BICYCLE = 4,
  STATIC = 5,
};

static std::string Name(const ObjectClass &type) {
  switch (type) {
    case ObjectClass::CAR:
      return "CAR";
    case ObjectClass::TRUCK:
      return "TRUCK";
    case ObjectClass::PEOPLE:
      return "PEOPLE";
    case ObjectClass::BICYCLE:
      return "BICYCLE";
    case ObjectClass::STATIC:
      return "STATIC";
    default:
      return "UNKNOW";
  }
}

enum class ObjectPathClass {
  UNKNOW = 0,
  LINE = 1,
  CURVE = 2,
  LANE = 3,
};

struct TrajectoryPoint {
  TrajectoryPoint() {}
  TrajectoryPoint(double _x, double _y, double _theta, double _v, double _t)
      : x(_x), y(_y), theta(_theta), v(_v), t(_t) {}
  double x;
  double y;
  double theta;
  double v;
  double t;
  double s;
  double kappa;
};

struct Object {
  Object() = default;
  Object(const geometry_msgs::msg::PoseWithCovarianceStamped &psd);
  Object(const Object& obj);
  ~Object() = default;
  void Reset();

  int id;
  ObjectClass object_class;
  double start_v;
  builtin_interfaces::msg::Time start_time;
  double life_time;
  ObjectPathClass object_path_class;
  double length;
  double width;
  double height;
  double start_x;
  double start_y;
  double start_heading;

  double remaining_time;
  double x;
  double y;
  double heading;
  double v;

  double traj_time = 8.0;
  double traj_time_resolution = 0.1;
  std::vector<TrajectoryPoint> trajectory;

  int nearest_lane_id = -1;
  double vehicle_x;
  double vehicle_y;
  double vehicle_heading;
};

}  // namespace simulator
}  // namespace xju
