/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include <cstdlib>
#include <memory>
#include <boost/program_options.hpp>

#include "rclcpp/rclcpp.hpp"
#include "planning/interface/ros/planning_node.h"

#ifndef VERSION
#define VERSION "UNKNOWN"
#endif


int main(int argc, char** argv) {
  std::cout << "version: " << VERSION << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto planning_node = std::make_shared<PlanningNode>("planning");
  executor.add_node(planning_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}