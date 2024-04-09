/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "goal_tool.hpp"

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rviz_common/display_context.hpp"
#include "rviz_common/logging.hpp"
#include "rviz_common/properties/qos_profile_property.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace xju {
namespace simulator {

GoalTool::GoalTool() : xju::simulator::PoseTool(), qos_profile_(5) {
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty(
      "Topic", "pose_grabber", "The topic on which to publish goals.",
      getPropertyContainer(), SLOT(updateTopic()), this);

  qos_profile_property_ = new rviz_common::properties::QosProfileProperty(
      topic_property_, qos_profile_);
}

GoalTool::~GoalTool() = default;

void GoalTool::onInitialize() {
  PoseTool::onInitialize();
  qos_profile_property_->initialize(
      [this](rclcpp::QoS profile) { this->qos_profile_ = profile; });
  setName("Pose Grabber");
  updateTopic();
}

void GoalTool::updateTopic() {
  rclcpp::Node::SharedPtr raw_node =
      context_->getRosNodeAbstraction().lock()->get_raw_node();
  // available
  publisher_ =
      raw_node->template create_publisher<geometry_msgs::msg::PoseStamped>(
          topic_property_->getStdString(), qos_profile_);
  clock_ = raw_node->get_clock();
}

void GoalTool::onPoseSet(double x, double y, double theta) {
  std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped goal;
  goal.header.stamp = clock_->now();
  goal.header.frame_id = fixed_frame;

  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = 0.0;

  goal.pose.orientation = orientationAroundZAxis(theta);

  logPose("goal", goal.pose.position, goal.pose.orientation, theta,
          fixed_frame);

  publisher_->publish(goal);
}

} // namespace simulator
} // namespace xju

#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(xju::simulator::GoalTool, rviz_common::Tool)
