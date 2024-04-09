/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

#include "pose_tool.hpp"
#include "visibility_control.hpp"

namespace rviz_common {
class DisplayContext;
namespace properties {
class StringProperty;
class QosProfileProperty;
} // namespace properties
} // namespace rviz_common

namespace xju {
namespace simulator {
class RVIZ_XJU_PLUGINS_PUBLIC GoalTool : public PoseTool {
  Q_OBJECT

public:
  GoalTool();

  ~GoalTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;

  rviz_common::properties::StringProperty *topic_property_;
  rviz_common::properties::QosProfileProperty *qos_profile_property_;

  rclcpp::QoS qos_profile_;
};

} // namespace simulator
} // namespace xju
