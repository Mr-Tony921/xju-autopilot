/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/logger/logger.h"
#include "localization_msgs/msg/localize_output.hpp"
#include "perception_em_msgs/msg/em_lanes.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simulator/common/hdmap_interface/hdmap_interface.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace xju {
namespace simulator {

struct pose_t {
  double x;
  double y;
  double z;
  double yaw;
};

class HDMapServerNode : public rclcpp::Node {
  static constexpr double START_DIST = -50.0;
  static constexpr double END_DIST = 200.0;

 public:
  HDMapServerNode() : Node("emlanes_simulator"), loc_received_(false) {
    hdmap_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            // "/hdmap/viz/hdmap_marker", 1);
            "/hdmap/viz/hdmap_marker",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    em_lane_pub_ =
        this->create_publisher<perception_em_msgs::msg::EmLanes>("/perception/em/lanes", 1);

    localization_sub_ =
        this->create_subscription<localization_msgs::msg::LocalizeOutput>(
            "/localization/output", 10,
            std::bind(&HDMapServerNode::LocalizationCallback, this,
                      std::placeholders::_1));

    InitParam();
    hdmap_interface_ = HDMapInterface::GetInstance();
    if (!hdmap_interface_->InitHDMap(hdmap_config_file_.c_str())) {
      AERROR << "HDMap init failed !";
    }

    PublishHDMapMarker();

    em_lane_timer_ = rclcpp::create_timer(
        this, this->get_clock(), std::chrono::milliseconds(50),
        std::bind(&HDMapServerNode::EMLaneTimerCallback, this));
  }

  ~HDMapServerNode() = default;

 private:
  void InitParam();
  void LocalizationCallback(
      const localization_msgs::msg::LocalizeOutput::SharedPtr msg);
  void PublishHDMapMarker();
  void EMLaneTimerCallback();
  auto GetLanePoints(int current_id, uint8_t line_choise) // right:0 center:1 left:2
      -> std::vector<common_msgs::msg::Point3D>;
  void GetCurveFromPts(const std::vector<common_msgs::msg::Point3D>& points,
                       perception_em_msgs::msg::CubicCurve* const curve);

 private:
  rclcpp::Subscription<localization_msgs::msg::LocalizeOutput>::SharedPtr
      localization_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      hdmap_marker_pub_;
  rclcpp::Publisher<perception_em_msgs::msg::EmLanes>::SharedPtr em_lane_pub_;
  rclcpp::TimerBase::SharedPtr em_lane_timer_;
  std::shared_ptr<HDMapInterface> hdmap_interface_;
  pose_t vehicle_pose_;
  std::string hdmap_config_file_;
  bool loc_received_;
};

}  // namespace simulator
}  // namespace xju
