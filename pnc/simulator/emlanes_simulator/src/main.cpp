/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "hdmap_server.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xju::simulator::HDMapServerNode>());
  rclcpp::shutdown();
  return 0;
}