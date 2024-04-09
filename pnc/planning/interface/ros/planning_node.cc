/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/interface/ros/planning_node.h"

#include <chrono>
#include <iostream>

#include "common/file/file.h"
#include "common/interface/ros/conversion.h"
#include "common/logger/glog_helper.h"
#include "common/logger/logger.h"
#include "common/time/time.h"
#include "common/util/coordinate.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "pnc_map/data_pool.h"
#include "std_msgs/msg/color_rgba.hpp"

PlanningNode::PlanningNode(const std::string& name) : Node(name) {
  DeclareParameter();
  // First Load Yaml file!
  LoadYamlConfig();
  // Second Init glog!
  xju::pnc::File::GetGflagConfig(gflag_config_file_path_);
  ResetFilePath();
  static std::shared_ptr<xju::pnc::GlogHelper> gh_ptr{
      new xju::pnc::GlogHelper{"planning"}};
  AINFO << name << " Node Created!";

  auto GenerateCallbackGroup = [this]() {
    return this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
  };

  // Create Subscriptions
  // 1. Subscripte prediction
  auto msg_opt = rclcpp::SubscriptionOptions();
  msg_opt.callback_group = GenerateCallbackGroup();
  std::string prediction_topic = "/perception/em/prediction";
  sub_prediction_ =
      this->create_subscription<perception_em_msgs::msg::PredictionObstacles>(
          prediction_topic, 1,
          std::bind(&PlanningNode::prediction_callback, this,
                    std::placeholders::_1),
          msg_opt);

  // 2. Subscripte em lanes
  std::string emlanes_topic = "/perception/em/lanes";
  sub_emlanes_ = this->create_subscription<perception_em_msgs::msg::EmLanes>(
      emlanes_topic, 1,
      std::bind(&PlanningNode::emlanes_callback, this, std::placeholders::_1),
      msg_opt);

  // 3. Subscripte localization
  std::string localization_topic = "/localization/output";
  sub_localization_ =
      this->create_subscription<localization_msgs::msg::LocalizeOutput>(
          localization_topic, 1,
          std::bind(&PlanningNode::localization_callback, this,
                    std::placeholders::_1),
          msg_opt);

  // 4. Subscripte chassis
  std::string brake_topic = "/data_interface/chassis/brake";
  sub_brake_ = this->create_subscription<chassis_msgs::msg::BrakeInfo>(
      brake_topic, 1,
      std::bind(&PlanningNode::brake_callback, this, std::placeholders::_1),
      msg_opt);

  std::string throttle_topic = "/data_interface/chassis/throttle";
  sub_throttle_ = this->create_subscription<chassis_msgs::msg::ThrottleInfo>(
      throttle_topic, 1,
      std::bind(&PlanningNode::throttle_callback, this, std::placeholders::_1),
      msg_opt);

  std::string steer_topic = "/data_interface/chassis/steer";
  sub_steer_ = this->create_subscription<chassis_msgs::msg::SteerInfo>(
      steer_topic, 1,
      std::bind(&PlanningNode::steer_callback, this, std::placeholders::_1),
      msg_opt);

  std::string vehicle_speed_topic = "/data_interface/chassis/vehicle_speed";
  sub_vehicle_speed_ =
      this->create_subscription<chassis_msgs::msg::CarSpeedInfo>(
          vehicle_speed_topic, 1,
          std::bind(&PlanningNode::vehicle_speed_callback, this,
                    std::placeholders::_1),
          msg_opt);

  std::string vehicle_mass_topic = "/data_interface/chassis/vehicle_mass";
  sub_vehicle_mass_ =
      this->create_subscription<chassis_msgs::msg::CarMassInfo>(
          vehicle_mass_topic, 1,
          std::bind(&PlanningNode::vehicle_mass_callback, this,
                    std::placeholders::_1),
          msg_opt);

  std::string gear_box_topic = "/data_interface/chassis/gear";
  sub_gear_box_ = this->create_subscription<chassis_msgs::msg::GearBoxInfo>(
      gear_box_topic, 1,
      std::bind(&PlanningNode::gear_box_callback, this, std::placeholders::_1),
      msg_opt);

  // 5. Subscripte destination
  std::string destination_topic = "/pnc/destination";
  sub_destination_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      destination_topic, 1,
      std::bind(&PlanningNode::destination_callback, this,
                std::placeholders::_1),
      msg_opt);
  // 6. Subscripte destination
  std::string routing_topic = "/map/routing";
  sub_routing_ = this->create_subscription<routing_msgs::msg::RoutingInfo>(
      routing_topic, 1,
      std::bind(&PlanningNode::routing_callback, this, std::placeholders::_1),
      msg_opt);

  // Create Publisher
  std::string planning_topic = "/pnc/planning";
  pub_planning_ =
      this->create_publisher<planning_msgs::msg::Planning>(planning_topic, 10);

  std::string debug_marker_topic = "/pnc/debug_marker";
  pub_debug_marker_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          debug_marker_topic, 1);

  // Create Timer
  static auto timer_callback_group = GenerateCallbackGroup();
  timer_ = rclcpp::create_timer(
      this, this->get_clock(), std::chrono::milliseconds(100),
      std::bind(&PlanningNode::Run, this), timer_callback_group);

  InitLocalView();
  // Init planner
  planning_.Init(proto_config_file_path_);
}

void PlanningNode::InitLocalView() {
  local_view_.prediction_obstacles =
      std::make_shared<xju::pnc::PredictionObstacles>();
  local_view_.chassis_info = std::make_shared<xju::pnc::ChassisInfo>();
  local_view_.localization = std::make_shared<xju::pnc::Localization>();
  local_view_.em_lanes = std::make_shared<xju::pnc::EmLanes>();
  local_view_.routing = std::make_shared<xju::pnc::Routing>();
}

void PlanningNode::DeclareParameter() {
  this->declare_parameter<std::string>("proto_config_file_path", "");
  this->declare_parameter<std::string>("gflag_config_file_path", "");
  this->declare_parameter<std::string>("scenario_lane_follow_config_file", "");
  this->declare_parameter<std::string>("scenario_road_change_config_file", "");
  this->declare_parameter<std::string>("scenario_emergency_pull_over_config_file", "");
  this->declare_parameter<std::string>("scenario_emergency_stop_config_file", "");
  this->declare_parameter<std::string>("lane_decider_default_config_file", "");
  this->declare_parameter<std::string>("vehicle_config_file", "");
  this->declare_parameter<std::string>("log_dir", "");
}

void PlanningNode::LoadYamlConfig() {
  this->get_parameter("proto_config_file_path", proto_config_file_path_);
  this->get_parameter("gflag_config_file_path", gflag_config_file_path_);
  this->get_parameter("scenario_lane_follow_config_file",
                      scenario_lane_follow_config_file_);
  this->get_parameter("scenario_road_change_config_file",
                      scenario_road_change_config_file_);
  this->get_parameter("scenario_emergency_pull_over_config_file",
                      scenario_emergency_pull_over_config_file_);
  this->get_parameter("scenario_emergency_stop_config_file",
                      scenario_emergency_stop_config_file_);
  this->get_parameter("lane_decider_default_config_file",
                      lane_decider_default_config_file_);
  this->get_parameter("vehicle_config_file", vehicle_config_file_);
  this->get_parameter("log_dir", log_dir_);
}

void PlanningNode::ResetFilePath() {
  xju::planning::FLAGS_scenario_lane_follow_config_file =
      scenario_lane_follow_config_file_;
  xju::planning::FLAGS_scenario_road_change_config_file =
      scenario_road_change_config_file_;
  xju::planning::FLAGS_scenario_emergency_pull_over_config_file =
      scenario_emergency_pull_over_config_file_;
  xju::planning::FLAGS_scenario_emergency_stop_config_file =
      scenario_emergency_stop_config_file_;
  xju::planning::FLAGS_lane_decider_default_config_file =
      lane_decider_default_config_file_;
  xju::pnc::FLAGS_vehicle_config_file = vehicle_config_file_;
  FLAGS_log_dir = log_dir_;
}

void PlanningNode::prediction_callback(
    const perception_em_msgs::msg::PredictionObstacles::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::prediction_obstacles_mutex);
  xju::pnc::PredictionConversion(msg,
                                     local_view_.prediction_obstacles.get());
}

void PlanningNode::emlanes_callback(
    const perception_em_msgs::msg::EmLanes::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::em_lanes_mutex);
  xju::pnc::EmLanesConversion(msg, local_view_.em_lanes.get());
}

void PlanningNode::localization_callback(
    const localization_msgs::msg::LocalizeOutput::SharedPtr msg) {
  // loc in vehicle coordinate
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::localization_mutex);
  xju::pnc::LocalizationConversion(msg, local_view_.localization.get());
}

void PlanningNode::brake_callback(
    const chassis_msgs::msg::BrakeInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::chassis_info_mutex);
  xju::pnc::BrakeInfoConversion(
      msg, local_view_.chassis_info->mutable_brake_info());
}

void PlanningNode::throttle_callback(
    const chassis_msgs::msg::ThrottleInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::chassis_info_mutex);
  xju::pnc::ThrottleInfoConversion(
      msg, local_view_.chassis_info->mutable_throttle_info());
}

void PlanningNode::steer_callback(
    const chassis_msgs::msg::SteerInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::chassis_info_mutex);
  xju::pnc::SteerInfoConversion(
      msg, local_view_.chassis_info->mutable_steer_info());
}

void PlanningNode::vehicle_speed_callback(
    const chassis_msgs::msg::CarSpeedInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::chassis_info_mutex);
  xju::pnc::CarSpeedInfoConversion(
      msg, local_view_.chassis_info->mutable_car_speed_info());
}

void PlanningNode::vehicle_mass_callback(
    const chassis_msgs::msg::CarMassInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::chassis_info_mutex);
  xju::pnc::CarMassInfoConversion(
      msg, local_view_.chassis_info->mutable_car_mass_info());
}

void PlanningNode::gear_box_callback(
    const chassis_msgs::msg::GearBoxInfo::SharedPtr msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::chassis_info_mutex);
  xju::pnc::GearBoxInfoConversion(
      msg, local_view_.chassis_info->mutable_gear_info());
}

void PlanningNode::destination_callback(
    const geometry_msgs::msg::PoseStamped msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::routing_mutex);
  common_msgs::msg::Point2D destination;
  destination.x = msg.pose.position.x;
  destination.y = msg.pose.position.y;
  xju::pnc::Point2DConversion(destination,
                                  local_view_.routing->mutable_destination());
  local_view_.routing->set_valid(true);
}

void PlanningNode::routing_callback(const routing_msgs::msg::RoutingInfo msg) {
  std::unique_lock<std::shared_mutex> lock(
      xju::planning::LocalView::routing_mutex);
  if (msg.e_mercator_x != 0 || msg.e_mercator_y != 0) {
    common_msgs::msg::Point2D destination;
    xju::pnc::Point2DConversion(destination,
                                    local_view_.routing->mutable_destination());
    local_view_.routing->set_valid(true);
  }
  if (msg.mercator_x != 0 || msg.mercator_y != 0) {
    common_msgs::msg::Point2D change_point;
    change_point.x = msg.mercator_x;
    change_point.y = msg.mercator_y;
    xju::pnc::Point2DConversion(
        change_point, local_view_.routing->mutable_change_point());
  }
  local_view_.routing->set_direction(static_cast<uint32_t>(msg.direction));
  local_view_.routing->mutable_header()->set_timestamp_sec(
      xju::pnc::Time::NowInSeconds());
}

void PlanningNode::PublishDebugMarker(
    const planning_msgs::msg::Planning& traj) {
  auto pnc_loc = xju::pnc::VehicleStateProvider::localize_pose();
  visualization_msgs::msg::MarkerArray debug_marker;
  //(1)traj marker
  visualization_msgs::msg::Marker traj_marker;
  traj_marker.header.frame_id = "local";
  // traj_marker.header.stamp = this->get_clock()->now();
  traj_marker.ns = "traj";
  if (debug_marker.markers.empty()) {
    traj_marker.id = 0;
  } else {
    traj_marker.id = debug_marker.markers.back().id + 1;
  }
  traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  traj_marker.action = visualization_msgs::msg::Marker::ADD;
  traj_marker.scale.x = 1.5;
  traj_marker.scale.y = 1.5;
  traj_marker.color.r = 1.0;
  traj_marker.color.g = 0.0;
  traj_marker.color.b = 0.0;
  traj_marker.color.a = 1.0;
  for (int i = 0; i < traj.trajectory_point.size(); i++) {
    geometry_msgs::msg::Point point;
    point.x = traj.trajectory_point[i].path_point.x;
    point.y = traj.trajectory_point[i].path_point.y;
    xju::pnc::TransformVehicleToLocalCoord(
        pnc_loc.x, pnc_loc.y, pnc_loc.heading, &point.x, &point.y);
    traj_marker.points.emplace_back(std::move(point));
    std_msgs::msg::ColorRGBA color;
    static const double max_v_threshold = 28.0;
    static const double min_v_threshold = 1.0;
    double val = std::max(std::min(traj.trajectory_point[i].v, max_v_threshold),
                          min_v_threshold);
    double rate =
        1.0 - (val - min_v_threshold) / (max_v_threshold - min_v_threshold);
    xju::pnc::ColorBar::Instance()->GetRGBRate(rate, &(color.r), &(color.g),
                                                   &(color.b));
    color.a = 0.8;
    traj_marker.colors.push_back(std::move(color));
  }
  debug_marker.markers.push_back(traj_marker);

  //(2)refline marker
  auto reflines = xju::pnc_map::DataPool::Instance()->reference_lines();
  for (auto iter = reflines.begin(); iter != reflines.end(); iter++) {
    visualization_msgs::msg::Marker ref_marker;
    ref_marker.header.frame_id = "local";
    // ref_marker.header.stamp = this->get_clock()->now();
    ref_marker.ns = "refline";
    if (debug_marker.markers.empty()) {
      ref_marker.id = 0;
    } else {
      ref_marker.id = debug_marker.markers.back().id + 1;
    }
    ref_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ref_marker.action = visualization_msgs::msg::Marker::ADD;
    ref_marker.scale.x = 0.1;
    ref_marker.scale.y = 0.1;
    ref_marker.scale.z = 0.1;
    ref_marker.color.r = 0.0;
    ref_marker.color.g = 1.0;
    ref_marker.color.b = 0.0;
    ref_marker.color.a = 0.8;
    const auto& ref_points = iter->reference_points();
    for (int i = 0; i < ref_points.size(); i++) {
      geometry_msgs::msg::Point point;
      point.x = ref_points[i].x();
      point.y = ref_points[i].y();
      xju::pnc::TransformVehicleToLocalCoord(
          pnc_loc.x, pnc_loc.y, pnc_loc.heading, &point.x, &point.y);
      ref_marker.points.emplace_back(std::move(point));
    }
    debug_marker.markers.push_back(ref_marker);
  }

  pub_debug_marker_->publish(debug_marker);
  xju::pnc_map::DataPool::Instance()->clear();
}

int PlanningNode::Run() {
  AINFO << "========================================"
           "========================================";
  xju::planning::ADCTrajectory trajectory_pb;
  planning_.Process(local_view_, &trajectory_pb);
  planning_msgs::msg::Planning planning_msg;
  xju::pnc::PlanningConversion(trajectory_pb, &planning_msg);
  pub_planning_->publish(planning_msg);
  PublishDebugMarker(planning_msg);
  AINFO << std::fixed << "traj_timestamp: "
        << xju::pnc::Time::NowInDateWithSec(
               planning_msg.header.timestamp_sec);
  return 0;
}
