/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include <string>

#include "common/interface/ros/conversion.h"

namespace xju {
namespace pnc {

void HeaderConversion(const common_msgs::msg::Header& msg,
                      Header* const proto) {
  proto->set_timestamp_sec(msg.timestamp_sec);
  proto->set_module_name(msg.module_name);
  proto->set_sequence_num(msg.sequence_num);
  proto->set_lidar_timestamp(msg.lidar_timestamp);
  proto->set_camera_timestamp(msg.camera_timestamp);
  proto->set_radar_timestamp(msg.radar_timestamp);
  proto->set_version(msg.version);
  proto->set_frame_id(msg.frame_id);
}

void HeaderConversion(const Header& proto,
                      common_msgs::msg::Header* const msg) {
  if (proto.has_timestamp_sec()) {
    msg->timestamp_sec = proto.timestamp_sec();
  }
  if (proto.has_module_name()) {
    msg->module_name = proto.module_name();
  }
  if (proto.has_sequence_num()) {
    msg->sequence_num = proto.sequence_num();
  }
  if (proto.has_lidar_timestamp()) {
    msg->lidar_timestamp = proto.lidar_timestamp();
  }
  if (proto.has_camera_timestamp()) {
    msg->camera_timestamp = proto.camera_timestamp();
  }
  if (proto.has_radar_timestamp()) {
    msg->radar_timestamp = proto.radar_timestamp();
  }
  if (proto.has_version()) {
    msg->version = proto.version();
  }
  if (proto.has_frame_id()) {
    msg->frame_id = proto.frame_id();
  }
}

void Point2DConversion(const common_msgs::msg::Point2D& msg,
                       Point2D* const proto) {
  proto->set_x(msg.x);
  proto->set_y(msg.y);
}

void Point2DConversion(
    const Point2D& proto,
    common_msgs::msg::Point2D* msg) {
  if (proto.has_x()) {
    msg->x = proto.x();
  }
  if (proto.has_y()) {
    msg->y = proto.y();
  }
}

void Vector3DConversion(const common_msgs::msg::Vector3D& msg,
                        Point3D* const proto) {
  proto->set_x(msg.x);
  proto->set_y(msg.y);
  proto->set_z(msg.z);
}

void Point3DConversion(const common_msgs::msg::Point3D& msg,
                       Point3D* const proto) {
  proto->set_x(msg.x);
  proto->set_y(msg.y);
  proto->set_z(msg.z);
}

void PathPointConversion(const common_msgs::msg::PathPoint& msg,
                         PathPoint* const proto) {
  proto->set_x(msg.x);
  proto->set_y(msg.y);
  proto->set_z(msg.z);
  proto->set_theta(msg.theta);
  proto->set_kappa(msg.kappa);
  proto->set_dkappa(msg.dkappa);
  proto->set_ddkappa(msg.ddkappa);
  proto->set_s(msg.s);
}

void PathPointConversion(const PathPoint& proto,
                         common_msgs::msg::PathPoint* const msg) {
  if (proto.has_x()) {
    msg->x = proto.x();
  }
  if (proto.has_y()) {
    msg->y = proto.y();
  }
  if (proto.has_z()) {
    msg->z = proto.z();
  }
  if (proto.has_theta()) {
    msg->theta = proto.theta();
  }
  if (proto.has_kappa()) {
    msg->kappa = proto.kappa();
  }
  if (proto.has_dkappa()) {
    msg->dkappa = proto.dkappa();
  }
  if (proto.has_ddkappa()) {
    msg->ddkappa = proto.ddkappa();
  }
  if (proto.has_s()) {
    msg->s = proto.s();
  }
}

void TrajectoryPointConversion(const common_msgs::msg::TrajectoryPoint& msg,
                               TrajectoryPoint* const proto) {
  PathPointConversion(msg.path_point, proto->mutable_path_point());
  proto->set_v(msg.v);
  proto->set_a(msg.a);
  proto->set_da(msg.da);
  proto->set_relative_time(msg.relative_time);
}

void TrajectoryPointConversion(const TrajectoryPoint& proto,
                               common_msgs::msg::TrajectoryPoint* const msg) {
  if (proto.has_path_point()) {
    PathPointConversion(proto.path_point(), &(msg->path_point));
  }
  if (proto.has_v()) {
    msg->v = proto.v();
  }
  if (proto.has_a()) {
    msg->a = proto.a();
  }
  if (proto.has_da()) {
    msg->da = proto.da();
  }
  if (proto.has_relative_time()) {
    msg->relative_time = proto.relative_time();
  }
}

}  // namespace pnc
}  // namespace xju
