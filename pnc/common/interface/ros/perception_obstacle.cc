/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/interface/ros/conversion.h"

namespace xju {
namespace pnc {

void PerceptionObstacleConversion(
    const perception_em_msgs::msg::EmObstacle& msg,
    PerceptionObstacle* const proto_ptr) {
  proto_ptr->set_id(msg.id);
  Point3DConversion(msg.position, proto_ptr->mutable_position());
  proto_ptr->set_heading(msg.theta);
  Point3DConversion(msg.velocity, proto_ptr->mutable_velocity());
  Point3DConversion(msg.acceleration, proto_ptr->mutable_acceleration());
  proto_ptr->set_length(msg.length);
  proto_ptr->set_width(msg.width);
  proto_ptr->set_height(msg.height);
  
  proto_ptr->clear_polygon_point();
  for (const auto& p : msg.polygon_point) {
    auto polygon_point = proto_ptr->add_polygon_point();
    Point3DConversion(p, polygon_point);
  }

  proto_ptr->set_type(static_cast<PerceptionObstacle::Type>(msg.type));
  proto_ptr->set_sub_type(
      static_cast<PerceptionObstacle::SubType>(msg.sub_type));
  // proto_ptr->set_height_above_ground(0.0);

  proto_ptr->clear_lane_ids();
  for (const auto& id : msg.lane_ids) {
    proto_ptr->add_lane_ids(id);
  }
}

}  // namespace pnc
}  // namespace xju