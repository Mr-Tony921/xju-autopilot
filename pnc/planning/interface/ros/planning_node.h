/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>

#include "planning/common/local_view.h"
#include "planning/planning.h"
#include "rclcpp/rclcpp.hpp"
// ros interface msgs
#include "chassis_msgs/msg/brake_info.hpp"
#include "chassis_msgs/msg/gear_box_info.hpp"
#include "chassis_msgs/msg/steer_info.hpp"
#include "chassis_msgs/msg/throttle_info.hpp"
#include "chassis_msgs/msg/car_mass_info.hpp"
#include "chassis_msgs/msg/car_speed_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "localization_msgs/msg/localize_output.hpp"
#include "perception_em_msgs/msg/em_lanes.hpp"
#include "perception_em_msgs/msg/prediction_obstacles.hpp"
#include "routing_msgs/msg/routing_info.hpp"
#include "planning_msgs/msg/planning.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "common/util/color_bar.h"

class PlanningNode : public rclcpp::Node {
 public:
  PlanningNode(const std::string& name);
  ~PlanningNode() = default;

 private:
  void DeclareParameter();
  void LoadYamlConfig();
  void ResetFilePath();
  int Run();
  void InitLocalView();

 private:
  void prediction_callback(
      const perception_em_msgs::msg::PredictionObstacles::SharedPtr msg);
  void emlanes_callback(const perception_em_msgs::msg::EmLanes::SharedPtr msg);

  void localization_callback(
      const localization_msgs::msg::LocalizeOutput::SharedPtr msg);

  void brake_callback(const chassis_msgs::msg::BrakeInfo::SharedPtr msg);
  void throttle_callback(const chassis_msgs::msg::ThrottleInfo::SharedPtr msg);
  void steer_callback(const chassis_msgs::msg::SteerInfo::SharedPtr msg);
  void vehicle_speed_callback(
      const chassis_msgs::msg::CarSpeedInfo::SharedPtr msg);
  void vehicle_mass_callback(
      const chassis_msgs::msg::CarMassInfo::SharedPtr msg);
  void gear_box_callback(const chassis_msgs::msg::GearBoxInfo::SharedPtr msg);

  void destination_callback(const geometry_msgs::msg::PoseStamped msg);

  void routing_callback(const routing_msgs::msg::RoutingInfo msg);

  void PublishDebugMarker(const planning_msgs::msg::Planning& traj);

 private:
  // Subscription
  rclcpp::Subscription<perception_em_msgs::msg::PredictionObstacles>::SharedPtr
      sub_prediction_;
  rclcpp::Subscription<perception_em_msgs::msg::EmLanes>::SharedPtr sub_emlanes_;
  rclcpp::Subscription<localization_msgs::msg::LocalizeOutput>::SharedPtr
      sub_localization_;
  // chassis info
  rclcpp::Subscription<chassis_msgs::msg::BrakeInfo>::SharedPtr sub_brake_;
  rclcpp::Subscription<chassis_msgs::msg::ThrottleInfo>::SharedPtr
      sub_throttle_;
  rclcpp::Subscription<chassis_msgs::msg::SteerInfo>::SharedPtr sub_steer_;
  rclcpp::Subscription<chassis_msgs::msg::CarSpeedInfo>::SharedPtr
      sub_vehicle_speed_;
  rclcpp::Subscription<chassis_msgs::msg::CarMassInfo>::SharedPtr
      sub_vehicle_mass_;
  rclcpp::Subscription<chassis_msgs::msg::GearBoxInfo>::SharedPtr sub_gear_box_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      sub_destination_;
  rclcpp::Subscription<routing_msgs::msg::RoutingInfo>::SharedPtr
      sub_routing_;
      
  // Publisher
  rclcpp::Publisher<planning_msgs::msg::Planning>::SharedPtr pub_planning_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pub_debug_marker_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  xju::planning::Planning planning_;
  xju::planning::LocalView local_view_;

  std::string proto_config_file_path_;
  std::string gflag_config_file_path_;
  std::string scenario_lane_follow_config_file_;
  std::string scenario_road_change_config_file_;
  std::string scenario_emergency_pull_over_config_file_;
  std::string scenario_emergency_stop_config_file_;
  std::string lane_decider_default_config_file_;
  std::string vehicle_config_file_;
  std::string log_dir_;
};
