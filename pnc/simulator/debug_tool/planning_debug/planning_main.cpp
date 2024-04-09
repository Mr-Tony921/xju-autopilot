/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning_debug_tool.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xju::simulator::DebugTool>());
  rclcpp::shutdown();
  return 0;
}