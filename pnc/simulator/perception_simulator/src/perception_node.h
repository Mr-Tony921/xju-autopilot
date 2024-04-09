/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <vector>

#include "common/logger/logger.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "localization_msgs/msg/localize_output.hpp"
#include "object.h"
#include "perception_em_msgs/msg/em_obstacles.hpp"
#include "perception_em_msgs/msg/prediction_obstacles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simulator/common/hdmap_interface/hdmap_interface.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace xju {
namespace simulator {

class PerceptionNode : public rclcpp::Node {
 public:
  PerceptionNode() : Node("perception_simulator") {
    loc_sub_ =
        this->create_subscription<localization_msgs::msg::LocalizeOutput>(
            "/localization/output", 10,
            std::bind(&PerceptionNode::LocCallback, this,
                      std::placeholders::_1));

    object_command_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/rviz_xju_panel/command", 10,
        std::bind(&PerceptionNode::ObjectCommandCallback, this,
                  std::placeholders::_1));

    object_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perception/viz/object", 1);

    pred_object_pub_ =
        this->create_publisher<perception_em_msgs::msg::PredictionObstacles>(
             "/perception/em/prediction", 1);

    em_object_pub_ = this->create_publisher<perception_em_msgs::msg::EmObstacles>(
        "/perception/em/obstacles", 1);

    object_timer_ = rclcpp::create_timer(
        this, this->get_clock(), std::chrono::milliseconds(50),
        std::bind(&PerceptionNode::ObjectTimerCallback, this));

    InitParam();
    hdmap_interface_ = HDMapInterface::GetInstance();
    if (!hdmap_interface_->InitHDMap(hdmap_config_file_.c_str())) {
      AERROR << "HDMap init failed !";
    }
  }
  ~PerceptionNode() = default;

 private:
  void InitParam();
  void ObjectTimerCallback();
  void UpdateObject();
  void PublishObject();
  void PublishObjectMarker();
  void ObjectCommandCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void LocCallback(const localization_msgs::msg::LocalizeOutput::SharedPtr msg);
  perception_em_msgs::msg::PredictionObstacles GeneratePredObs(
      const std::vector<std::shared_ptr<Object>>& objs);
  perception_em_msgs::msg::EmObstacles GenerateEmObs(
      const std::vector<std::shared_ptr<Object>>& objs);

 private:
  rclcpp::Publisher<perception_em_msgs::msg::PredictionObstacles>::SharedPtr
      pred_object_pub_;
  rclcpp::Publisher<perception_em_msgs::msg::EmObstacles>::SharedPtr
      em_object_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      object_marker_pub_;
  rclcpp::TimerBase::SharedPtr object_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      object_command_sub_;
  rclcpp::Subscription<localization_msgs::msg::LocalizeOutput>::SharedPtr
      loc_sub_;
  std::unordered_map<int, std::shared_ptr<Object>> objects_;
  int old_obj_size_ = 0;
  builtin_interfaces::msg::Time sim_time_;
  double loc_x_ = 0.0;
  double loc_y_ = 0.0;
  double loc_heading_ = 0.0;

  std::shared_ptr<HDMapInterface> hdmap_interface_;
  std::string hdmap_config_file_;
  bool publish_em_obs_ = false;
};

}  // namespace simulator
}  // namespace xju
