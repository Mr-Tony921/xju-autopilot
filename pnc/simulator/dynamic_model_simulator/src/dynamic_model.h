/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "chassis_msgs/msg/brake_info.hpp"
#include "chassis_msgs/msg/gear_box_info.hpp"
#include "chassis_msgs/msg/steer_info.hpp"
#include "chassis_msgs/msg/throttle_info.hpp"
#include "chassis_msgs/msg/car_mass_info.hpp"
#include "chassis_msgs/msg/car_speed_info.hpp"
#include "chassis_msgs/msg/wheel_speed_info.hpp"
#include "common/logger/logger.h"
#include "common/logger/glog_helper.h"
#include "common/time/time.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common_msgs/msg/vehicle_signal.hpp"
#include "control_msgs/msg/control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "localization_msgs/msg/localize_output.hpp"
#include "planning_msgs/msg/planning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

namespace xju {
namespace simulator {

enum class ControlMode { PERFECT = 0, KINEMATICS = 1, TORQUE = 2 };

class DynamicModelNode : public rclcpp::Node {
 public:
  DynamicModelNode() : Node("dynamic_model_simulator") {
    InitParam();

    Reset();

    // log
    static std::shared_ptr<xju::pnc::GlogHelper> gh_ptr{
        new xju::pnc::GlogHelper{"dynamic_model"}};

    // pub
    loc_pub_ = this->create_publisher<localization_msgs::msg::LocalizeOutput>(
        "/localization/output", 1);

    chassis_brakeinfo_pub_ =
        this->create_publisher<chassis_msgs::msg::BrakeInfo>(
            "/data_interface/chassis/brake", 1);

    chassis_gearboxinfo_pub_ =
        this->create_publisher<chassis_msgs::msg::GearBoxInfo>(
            "/data_interface/chassis/gear", 1);

    chassis_throttleinfo_pub_ =
        this->create_publisher<chassis_msgs::msg::ThrottleInfo>(
            "/data_interface/chassis/throttle", 1);

    chassis_carspeedinfo_pub_ =
        this->create_publisher<chassis_msgs::msg::CarSpeedInfo>(
            "/data_interface/chassis/vehicle_speed", 1);

    chassis_wheelspeedinfo_pub_ =
        this->create_publisher<chassis_msgs::msg::WheelSpeedInfo>(
            "/data_interface/chassis/wheel_speed", 1);

    chassis_carmassinfo_pub_ =
        this->create_publisher<chassis_msgs::msg::CarMassInfo>(
            "/data_interface/chassis/vehicle_mass", 1);

    chassis_steerinfo_pub_ =
        this->create_publisher<chassis_msgs::msg::SteerInfo>(
            "/data_interface/chassis/steer", 1);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/debug_tool/viz/ego_model", 1);

    // timer
    timer_ = rclcpp::create_timer(
        this, this->get_clock(), std::chrono::milliseconds(20),
        std::bind(&DynamicModelNode::TimerCallback, this));

    // sub
    reset_localization_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 10,
        std::bind(&DynamicModelNode::ResetLocalizationCallback, this,
                  std::placeholders::_1));

    reset_init_state_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rviz_xju_panel/command", 10,
        std::bind(&DynamicModelNode::ResetInitStateCallback, this,
                  std::placeholders::_1));

    planning_traj_sub_ =
        this->create_subscription<planning_msgs::msg::Planning>(
            "/pnc/planning", 10,
            std::bind(&DynamicModelNode::PlanningTrajCallback, this,
                      std::placeholders::_1));

    control_cmd_sub_ = this->create_subscription<control_msgs::msg::Control>(
        "/pnc/control", 10,
        std::bind(&DynamicModelNode::ControlCmdCallback, this,
                  std::placeholders::_1));
    // tf
    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  ~DynamicModelNode() = default;

 private:
  void InitParam();
  void Reset();
  void UpdateDynamicModel();
  void UpdateDynamicModelWithPerfectControl();
  void PlanningTrajCallback(const planning_msgs::msg::Planning::SharedPtr msg);
  void ControlCmdCallback(const control_msgs::msg::Control::SharedPtr msg);
  void ResetLocalizationCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void ResetInitStateCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void TimerCallback();
  double CalAccFromTorque(double torque);

  void PublishLoc();
  void PublishChassis();
  void PublishTF();
  void PublishCarMarker();

 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      reset_localization_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      reset_init_state_sub_;
  rclcpp::Subscription<planning_msgs::msg::Planning>::SharedPtr
      planning_traj_sub_;
  rclcpp::Subscription<control_msgs::msg::Control>::SharedPtr control_cmd_sub_;

  rclcpp::Publisher<localization_msgs::msg::LocalizeOutput>::SharedPtr loc_pub_;

  rclcpp::Publisher<chassis_msgs::msg::BrakeInfo>::SharedPtr
      chassis_brakeinfo_pub_;
  rclcpp::Publisher<chassis_msgs::msg::GearBoxInfo>::SharedPtr
      chassis_gearboxinfo_pub_;
  rclcpp::Publisher<chassis_msgs::msg::ThrottleInfo>::SharedPtr
      chassis_throttleinfo_pub_;
  rclcpp::Publisher<chassis_msgs::msg::CarSpeedInfo>::SharedPtr
      chassis_carspeedinfo_pub_;
  rclcpp::Publisher<chassis_msgs::msg::WheelSpeedInfo>::SharedPtr
      chassis_wheelspeedinfo_pub_;
  rclcpp::Publisher<chassis_msgs::msg::CarMassInfo>::SharedPtr
      chassis_carmassinfo_pub_;
  rclcpp::Publisher<chassis_msgs::msg::SteerInfo>::SharedPtr
      chassis_steerinfo_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      marker_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  double x_1_;
  double y_1_;
  double theta_1_;
  double l_1_;
  double v_x1_;

  double delta_;  // front wheel angle
  double d_;      // rear to hinge
  double v_p_;    // hinge speed

  double rear_to_back_ = 0.5;
  double car_length_ = 6.0;
  double car_width_ = 2.5;
  double car_height_ = 2.0;

  double min_vehicle_acc_ = -5.0;
  double max_vehicle_acc_ = 2.0;
  double min_vehicle_speed_ = -20.0 / 3.6;
  double max_vehicle_speed_ = 100 / 3.6;

  // command
  double control_cmd_time_stamp_ = -1.0;
  double wheel_angle_ = 0.0;  // rad
  double acc_cmd_ = 0.0;
  double steer_ratio_;

  double control_time_ = 1.0 / 50.0;  // default

  ControlMode control_mode_ = ControlMode::PERFECT;

  rclcpp::Time planning_traj_time_;

  std::unique_ptr<planning_msgs::msg::Planning> planning_traj_;

  // torque control param
  double R = 0;            // 轮胎半径
  double M = 0;            // 车重
  double g = 0.98;         // 重力加速度
  double f = 0.015;        // 滚阻
  double Ca = 0.8;         // 风阻系数
  double A = 8.5;          // 迎风面积
  double P_rou = 1.2258;   // 空气密度
  double theta_slope = 0;  // 坡度

  // default_start_loc
  double default_start_x_;
  double default_start_y_;
  double default_start_heading_;
};

}  // namespace simulator
}  // namespace xju
