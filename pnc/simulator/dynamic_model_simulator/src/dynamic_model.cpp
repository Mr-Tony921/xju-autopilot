
/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "dynamic_model.h"

#include <fstream>

#include "common/file/file.h"
#include "common/math/math_utils.h"
#include "common/util/coordinate.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace xju {
namespace simulator {

void DynamicModelNode::InitParam() {
  // load yaml param
  this->declare_parameter<std::string>("gflag_config_file_path", "");
  this->declare_parameter<std::string>("control_mode", "");
  this->declare_parameter<std::string>("vehicle_config_file", "");
  this->declare_parameter<std::string>("log_dir", "");
  this->declare_parameter<double>("start_x", 0.0);
  this->declare_parameter<double>("start_y", 0.0);
  this->declare_parameter<double>("start_heading", 0.0);
  std::string gflag_config_file_path, control_mode, vehicle_config_file;
  this->get_parameter("gflag_config_file_path", gflag_config_file_path);
  this->get_parameter("control_mode", control_mode);
  this->get_parameter("vehicle_config_file", vehicle_config_file);
  std::string log_dir;
  this->get_parameter("log_dir", log_dir);
  FLAGS_log_dir = log_dir;
  if (control_mode == "p") {
    control_mode_ = ControlMode::PERFECT;
    AINFO << "control_mode: perfect";
  } else if (control_mode == "k") {
    control_mode_ = ControlMode::KINEMATICS;
    AINFO << "control_mode: kinematics";
  } else if (control_mode == "t") {
    control_mode_ = ControlMode::TORQUE;
    AINFO << "control_mode: torque";
  } else {
    // default perfect
    AINFO << "control_mode: perfect";
  }
  
  this->get_parameter("start_x", default_start_x_);
  this->get_parameter("start_y", default_start_y_);
  this->get_parameter("start_heading", default_start_heading_);

  // init gflag param
  xju::pnc::File::GetGflagConfig(gflag_config_file_path);
  xju::pnc::FLAGS_vehicle_config_file = vehicle_config_file;
  // get vehicle config
  auto vehicle_config = xju::pnc::VehicleConfigProvider::GetConfig();

  d_ = vehicle_config.rear_axle_to_hitch();  // d>0 lizhou //d<0 guozhou
  rear_to_back_ = vehicle_config.back_overhang_length();
  car_length_ = vehicle_config.length();
  car_width_ = vehicle_config.width();
  car_height_ = vehicle_config.height();
  l_1_ = vehicle_config.wheel_base();
  steer_ratio_ = vehicle_config.steer_ratio();

  min_vehicle_acc_ = -vehicle_config.max_deceleration();
  max_vehicle_acc_ = vehicle_config.max_acceleration();
  min_vehicle_speed_ = vehicle_config.min_vehicle_speed() / 3.6;
  max_vehicle_speed_ = vehicle_config.max_vehicle_speed() / 3.6;

  R = vehicle_config.wheel_rolling_radius();
  M = vehicle_config.mass_min_limit() +
      vehicle_config.mass_max_limit();
  M /= 2.0;
}

void DynamicModelNode::Reset() {
  x_1_ = default_start_x_;
  y_1_ = default_start_y_;
  theta_1_ = default_start_heading_;
  v_x1_ = 0.0;

  delta_ = 0.0;
  v_p_ = 0.0;

  planning_traj_ = nullptr;
}

void DynamicModelNode::UpdateDynamicModelWithPerfectControl() {
  if (!planning_traj_) {
    v_x1_ = 0;
    acc_cmd_ = 0.0;
    return;
  }

  auto temp_time = this->get_clock()->now();
  auto t = (temp_time - planning_traj_time_).seconds();
  if (t >= 7.9) {
    planning_traj_->trajectory_point.clear();
  }
  const auto& traj_points = planning_traj_->trajectory_point;
  int size = traj_points.size();
  if (size == 0) {
    // x_1_ = 0.0;
    // y_1_ = 0.0;
    // theta_1_ = 0.0;
    v_x1_ = 0.0;
  } else if (size == 1) {
    const auto& point = traj_points.front();
    x_1_ = point.path_point.x;
    y_1_ = point.path_point.y;
    theta_1_ = point.path_point.theta;
    v_x1_ = 0;
    acc_cmd_ = 0.0;
  } else {
    for (int i = 0; i < size; i++) {
      if (traj_points[i].relative_time >= t) {
        x_1_ = traj_points[i].path_point.x;
        y_1_ = traj_points[i].path_point.y;
        theta_1_ = traj_points[i].path_point.theta;
        v_x1_ = traj_points[i].v;
        acc_cmd_ = traj_points[i].a;
        break;
      }
    }
  }
}

void DynamicModelNode::UpdateDynamicModel() {
  const auto t = control_time_;

  delta_ = wheel_angle_;

  double v_x = v_x1_ * std::cos(theta_1_);
  double acc_x = acc_cmd_ * std::cos(theta_1_);
  double v_y = v_x1_ * std::sin(theta_1_);
  double acc_y = acc_cmd_ * std::sin(theta_1_);
  double d_theta_1 = v_x1_ * std::tan(delta_) / l_1_;

  double cal_v = v_x1_ + acc_cmd_ * t;
  if (cal_v >= 0) {
    v_x1_ = cal_v;
  } else {
    v_x1_ = 0.0;
    v_x = 0.0;
    v_y = 0.0;
    acc_x = 0.0;
    acc_y = 0.0;
    d_theta_1 = 0.0;
  }

  x_1_ = x_1_ + v_x * t +
         1.0 / 2.0 * acc_x * t * t;  // update x_1_
  y_1_ = y_1_ + v_y * t +
         1.0 / 2.0 * acc_y * t * t;  // update y_1_
  theta_1_ += d_theta_1 * t;
}

void DynamicModelNode::ResetLocalizationCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  AINFO << "Receive reset loc command !";
  Reset();
  x_1_ = msg->pose.pose.position.x;
  y_1_ = msg->pose.pose.position.y;
  theta_1_ = tf2::getYaw(msg->pose.pose.orientation);
}

void DynamicModelNode::ResetInitStateCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (msg->header.frame_id != "reset_init_state") {
    return;
  }
  AINFO << "Receive reset init state command !";
  x_1_ = msg->pose.pose.position.x;
  y_1_ = msg->pose.pose.position.y;
  theta_1_ = msg->pose.pose.position.z;
  v_x1_ = msg->pose.pose.orientation.x;
  acc_cmd_ = msg->pose.pose.orientation.y;
}

void DynamicModelNode::TimerCallback() {
  if (control_mode_ == ControlMode::PERFECT) {
    UpdateDynamicModelWithPerfectControl();
  } else {
    // UpdateDynamicModel();
    if (acc_cmd_ < min_vehicle_acc_) {
      acc_cmd_ = min_vehicle_acc_;
    } else if (acc_cmd_ > max_vehicle_acc_) {
      acc_cmd_ = max_vehicle_acc_;
    }

    if (v_x1_ < min_vehicle_speed_) {
      v_x1_ = min_vehicle_speed_;
    } else if (v_x1_ > max_vehicle_speed_) {
      v_x1_ = max_vehicle_speed_;
    }
  }

  // publish loc
  PublishLoc();

  // publish chassis
  PublishChassis();

  // publish tf
  PublishTF();
}

void DynamicModelNode::PlanningTrajCallback(
    const planning_msgs::msg::Planning::SharedPtr msg) {
  planning_traj_time_ = this->get_clock()->now();
  if (!planning_traj_) {
    planning_traj_ = std::make_unique<planning_msgs::msg::Planning>(*msg);
  } else {
    *planning_traj_ = *msg;
  }
  // change vehicle to local
  double local_x, local_y, local_theta;
  for (std::size_t i = 0; i < planning_traj_->trajectory_point.size(); i++) {
    local_x = planning_traj_->trajectory_point[i].path_point.x;
    local_y = planning_traj_->trajectory_point[i].path_point.y;
    local_theta = planning_traj_->trajectory_point[i].path_point.theta;
    pnc::TransformVehicleToLocalCoord(x_1_, y_1_, theta_1_, &local_x, &local_y,
                                      &local_theta);
    planning_traj_->trajectory_point[i].path_point.x = local_x;
    planning_traj_->trajectory_point[i].path_point.y = local_y;
    planning_traj_->trajectory_point[i].path_point.theta = local_theta;
  }
}

double DynamicModelNode::CalAccFromTorque(double torque) {
  double val_1 = torque / R;
  double val_2 = Ca * P_rou * A * v_x1_ * v_x1_ / 2.0;
  double val_3 = M * g * std::sin(theta_slope);
  double val_4 = f * M * g * std::cos(theta_slope);
  double F = val_1 - (val_2 + val_3 + val_4);
  return F / M;
}

void DynamicModelNode::ControlCmdCallback(
    const control_msgs::msg::Control::SharedPtr msg) {
  const auto& now = pnc::Time::NowInSeconds();
  if (control_mode_ == ControlMode::TORQUE) {
    if (msg->wheel_torque > 0) {
      acc_cmd_ = CalAccFromTorque(msg->wheel_torque);
    } else {
      acc_cmd_ = msg->decelerate;
    }
  } else {
    acc_cmd_ = msg->acceleration;
  }
  wheel_angle_ = pnc::ToRadian(msg->steering / steer_ratio_);
  if (control_cmd_time_stamp_ > 0) {
    control_time_ = now - control_cmd_time_stamp_;
  }
  control_cmd_time_stamp_ = now;
  UpdateDynamicModel();
}

void DynamicModelNode::PublishLoc() {
  localization_msgs::msg::LocalizeOutput loc;
  static int loc_seq_num = 1;
  loc.header.sequence_num = loc_seq_num++;
  loc.header.timestamp_sec = pnc::Time::NowInSeconds();
  loc.header.module_name = "localization";
  loc.header.frame_id = "local";
  // loc.sequence
  loc.sys_time_us = pnc::Time::NowInSeconds();
  // loc.utc_time_sec =
  loc.local_localize_state = localization_msgs::msg::LocalizeOutput::TRACKING_L;
  // loc.local_localize_result.sequence
  loc.local_localize_result.sys_time_us = pnc::Time::NowInSeconds();  //?
  tf2::Quaternion temp_qua;
  temp_qua.setRPY(0.0, 0.0, theta_1_);
  loc.local_localize_result.rotation.qx = temp_qua.x();
  loc.local_localize_result.rotation.qy = temp_qua.y();
  loc.local_localize_result.rotation.qz = temp_qua.z();
  loc.local_localize_result.rotation.qw = temp_qua.w();
  loc.local_localize_result.euler_ypr.x = 0.0;
  loc.local_localize_result.euler_ypr.y = 0.0;
  loc.local_localize_result.euler_ypr.z = theta_1_;
  loc.local_localize_result.position.x = x_1_;
  loc.local_localize_result.position.y = y_1_;
  loc.local_localize_result.position.z = 0.0;
  loc.local_localize_result.velocity.x = v_x1_ * std::cos(theta_1_);
  loc.local_localize_result.velocity.y = v_x1_ * std::sin(theta_1_);
  loc.local_localize_result.velocity.z = 0.0;
  loc.local_localize_result.velocity_in_vehicle.x = v_x1_;
  loc.local_localize_result.velocity_in_vehicle.y = 0.0;
  loc.local_localize_result.velocity_in_vehicle.z = 0.0;
  loc.local_localize_result.angular_velocity_in_vehicle.x = 0.0;
  loc.local_localize_result.angular_velocity_in_vehicle.y = 0.0;
  loc.local_localize_result.angular_velocity_in_vehicle.z =
      v_x1_ * std::tan(delta_) / l_1_;
  loc.local_localize_result.linear_acceleration_in_vehicle.x = acc_cmd_;
  loc.local_localize_result.linear_acceleration_in_vehicle.y = 0.0;
  loc.local_localize_result.linear_acceleration_in_vehicle.z = 0.0;
  // loc.global_localize_result
  loc.global_localize_result.global_level.yaw = theta_1_;
  loc.global_localize_result.global_level.mercator_x = x_1_;
  loc.global_localize_result.global_level.mercator_y = y_1_;

  loc_pub_->publish(std::move(loc));
}

void DynamicModelNode::PublishChassis() {
  static int seq_num = 1;
  // brake_info
  chassis_msgs::msg::BrakeInfo brake_info;
  brake_info.header.sequence_num = seq_num;
  brake_info.header.timestamp_sec = pnc::Time::NowInSeconds();
  chassis_brakeinfo_pub_->publish(std::move(brake_info));
  // gear_box_info
  chassis_msgs::msg::GearBoxInfo gear_box_info;
  gear_box_info.header.sequence_num = seq_num;
  gear_box_info.header.timestamp_sec = pnc::Time::NowInSeconds();
  chassis_gearboxinfo_pub_->publish(std::move(gear_box_info));
  // throttle_info
  chassis_msgs::msg::ThrottleInfo throttle_info;
  throttle_info.header.sequence_num = seq_num;
  throttle_info.header.timestamp_sec = pnc::Time::NowInSeconds();
  chassis_throttleinfo_pub_->publish(std::move(throttle_info));
  // car_speed_info
  chassis_msgs::msg::CarSpeedInfo car_speed_info;
  car_speed_info.header.sequence_num = seq_num;
  car_speed_info.header.timestamp_sec = pnc::Time::NowInSeconds();
  car_speed_info.header.module_name = "canbus";
  car_speed_info.header.frame_id = "vehicle";
  car_speed_info.isvalid = true;
  car_speed_info.speed = v_x1_;
  car_speed_info.acceleration = acc_cmd_;
  car_speed_info.direction = chassis_msgs::msg::CarSpeedInfo::FORWARD;
  chassis_carspeedinfo_pub_->publish(std::move(car_speed_info));
  // wheel_speed_info
  chassis_msgs::msg::WheelSpeedInfo wheel_speed_info;
  wheel_speed_info.header.sequence_num = seq_num;
  wheel_speed_info.header.timestamp_sec = pnc::Time::NowInSeconds();
  chassis_wheelspeedinfo_pub_->publish(std::move(wheel_speed_info));
  // car_mass_info
  chassis_msgs::msg::CarMassInfo car_mass_info;
  car_mass_info.header.sequence_num = seq_num;
  car_mass_info.header.timestamp_sec = pnc::Time::NowInSeconds();
  chassis_carmassinfo_pub_->publish(std::move(car_mass_info));
  // steer_info
  chassis_msgs::msg::SteerInfo steer_info;
  steer_info.header.sequence_num = seq_num;
  steer_info.header.timestamp_sec = pnc::Time::NowInSeconds();
  chassis_steerinfo_pub_->publish(std::move(steer_info));

  seq_num++;
}

void DynamicModelNode::PublishTF() {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "local";
  t.child_frame_id = "vehicle";
  t.transform.translation.x = x_1_;
  t.transform.translation.y = y_1_;
  t.transform.translation.z = 0.4;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_1_);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_br_->sendTransform(t);

  PublishCarMarker();
}

void DynamicModelNode::PublishCarMarker() {
  // display car model
  visualization_msgs::msg::Marker car_marker;
  car_marker.header.frame_id = "vehicle";
  // car_marker.header.stamp = this->get_clock()->now();
  car_marker.ns = "vehicle";
  car_marker.id = 0;
  car_marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  car_marker.action = visualization_msgs::msg::Marker::ADD;
  car_marker.mesh_resource = "package://debug_tool_package/meshes/Lexus.dae";
  car_marker.scale.x = 1.5;
  car_marker.scale.y = 1.5;
  car_marker.scale.z = 1.5;
  car_marker.color.b = 0.0;
  car_marker.color.g = 140.0 / 255.0;
  car_marker.color.r = 255.0 / 255.0;
  car_marker.color.a = 1.0;

  car_marker.pose.position.x = 0.0;
  car_marker.pose.position.y = 0.0;
  car_marker.pose.position.z = -0.5;
  tf2::Quaternion temp_qua;
  temp_qua.setRPY(0.0, 0.0, -1.57);
  car_marker.pose.orientation.x = temp_qua.x();
  car_marker.pose.orientation.y = temp_qua.y();
  car_marker.pose.orientation.z = temp_qua.z();
  car_marker.pose.orientation.w = temp_qua.w();

  marker_pub_->publish(car_marker);
}

}  // namespace simulator
}  // namespace xju
