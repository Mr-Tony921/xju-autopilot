/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "common/file/file.h"
#include "common/gflags/global_gflags.h"
#include "common/logger/glog_helper.h"
#include "common/logger/logger.h"
#include "common/time/time.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common_msgs/msg/vehicle_signal.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "localization_msgs/msg/localize_output.hpp"
#include "planning_msgs/msg/planning.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_2d_overlay_msgs/msg/overlay_text.hpp"
#include "simulator/debug_tool/matplotlibcpp.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace plt = matplotlibcpp;

namespace xju {
namespace simulator {

class DebugTool : public rclcpp::Node {
 public:
  DebugTool() : Node("debug_tool") {
    InitParam();
    open_debug_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rviz_xju_panel/command", 1,
        std::bind(&DebugTool::SimCmdCallback, this, std::placeholders::_1));

    planning_traj_sub_ =
        this->create_subscription<planning_msgs::msg::Planning>(
            "/pnc/planning", 1,
            std::bind(&DebugTool::PlanningTrajCallback, this,
                      std::placeholders::_1));

    localization_sub_ =
        this->create_subscription<localization_msgs::msg::LocalizeOutput>(
            "/localization/output", 1,
            std::bind(&DebugTool::LocalizationCallback, this,
                      std::placeholders::_1));

    planning_overlay_pub_ =
        this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
            "/debug_tool/planning_overlay", 1);

    chassis_overlay_pub_ =
        this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
            "/debug_tool/chassis_overlay", 1);

    clock_pub_ =
        this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);

    marker_timer_ = rclcpp::create_timer(
        this, this->get_clock(), std::chrono::milliseconds(20),
        std::bind(&DebugTool::PublishOverlayText, this));

    std::thread chart_thread(std::bind(&DebugTool::GenerateChart, this));
    chart_thread.detach();

    if (publish_clock_) {
      real_time_ = this->get_clock()->now();
      msg_.clock = real_time_;
      std::thread clock_thread(&DebugTool::PublishClock, this);
      clock_thread.detach();
    }

    AINFO << "debug_tool Node Created!";
  }
  ~DebugTool() = default;

 private:
  void InitParam();
  void SimCmdCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void PlanningTrajCallback(const planning_msgs::msg::Planning::SharedPtr msg);

  void LocalizationCallback(
      const localization_msgs::msg::LocalizeOutput::SharedPtr msg);

  std::string ToColorKeyWord(const float r, const float g, const float b);
  std::string ToLineTypeKeyWord(const uint val);
  void PublishOverlayText();
  void GenerateChart();
  void PublishClock();

 private:
  rclcpp::TimerBase::SharedPtr marker_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      open_debug_sub_;
  rclcpp::Subscription<planning_msgs::msg::Planning>::SharedPtr
      planning_traj_sub_;
  rclcpp::Subscription<localization_msgs::msg::LocalizeOutput>::SharedPtr
      localization_sub_;
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr
      planning_overlay_pub_,
      chassis_overlay_pub_;
  bool figure_has_opened = false;
  int row_num = 0;
  int col_num = 0;
  bool open_debug = false;

  std::unique_ptr<planning_msgs::msg::Planning> planning_traj_ = nullptr;
  std::unique_ptr<localization_msgs::msg::LocalizeOutput> localization_ =
      nullptr;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  bool publish_clock_ = false;
  double sim_rate_ = 1.0;
  rclcpp::Time real_time_;
  rosgraph_msgs::msg::Clock msg_;

  std::string control_mode_;
  double rear_to_back_;
  double car_length_;
  double car_width_;
  double car_height_;
  double rear_axle_to_hitch_;
};

}  // namespace simulator
}  // namespace xju
