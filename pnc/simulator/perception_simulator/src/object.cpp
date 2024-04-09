
/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "object.h"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace xju {
namespace simulator {

Object::Object(const geometry_msgs::msg::PoseWithCovarianceStamped& psd) {
  id = (int)psd.pose.covariance[0];
  if ((int)psd.pose.covariance[8] == 0) {
    object_class = ObjectClass::CAR;
  } else if ((int)psd.pose.covariance[8] == 1) {
    object_class = ObjectClass::TRUCK;
  } else if ((int)psd.pose.covariance[8] == 2) {
    object_class = ObjectClass::PEOPLE;
  } else if ((int)psd.pose.covariance[8] == 3) {
    object_class = ObjectClass::BICYCLE;
  } else if ((int)psd.pose.covariance[8] == 4) {
    object_class = ObjectClass::STATIC;
  } else {
    object_class = ObjectClass::UNKNOW;
  }
  start_v = psd.pose.covariance[1];
  // start_time = psd.pose.covariance[3];
  life_time = psd.pose.covariance[2];
  if ((int)psd.pose.covariance[7] == 0) {
    object_path_class = ObjectPathClass::LANE;
  } else if ((int)psd.pose.covariance[7] == 1) {
    object_path_class = ObjectPathClass::LINE;
  } else if ((int)psd.pose.covariance[7] == 2) {
    object_path_class = ObjectPathClass::CURVE;
  } else {
    object_path_class = ObjectPathClass::UNKNOW;
  }

  length = psd.pose.covariance[4];
  width = psd.pose.covariance[5];
  height = psd.pose.covariance[6];
  start_x = psd.pose.pose.position.x;
  start_y = psd.pose.pose.position.y;
  start_heading = tf2::getYaw(psd.pose.pose.orientation);

  Reset();
}

Object::Object(const Object& obj) {
  id = obj.id;
  object_class = obj.object_class;
  start_v = obj.start_v;
  start_time = obj.start_time;
  life_time = obj.life_time;
  object_path_class = obj.object_path_class;
  length = obj.length;
  width = obj.width;
  height = obj.height;
  start_x = obj.start_x;
  start_y = obj.start_y;
  start_heading = obj.start_heading;

  remaining_time = obj.remaining_time;
  x = obj.x;
  y = obj.y;
  heading = obj.heading;
  v = obj.v;

  traj_time = obj.traj_time;
  traj_time_resolution = obj.traj_time_resolution;
  trajectory = obj.trajectory;

  nearest_lane_id = obj.nearest_lane_id;
}

void Object::Reset() {
  // start_time
  remaining_time = life_time;
  x = start_x;
  y = start_y;
  heading = start_heading;
  v = start_v;
  nearest_lane_id = -1;
}

}  // namespace simulator
}  // namespace xju
