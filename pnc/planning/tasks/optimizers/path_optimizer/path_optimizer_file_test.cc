/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/path_optimizer/path_optimizer.h"
#include "planning/tasks/optimizers/path_optimizer/vehicle_model_optimizer_osqp.h"

#include <fstream>

#include "common/math/math_utils.h"
#include "gtest/gtest.h"
#include "planning_config.pb.h"
#include "common/file/file.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "pnc_map/reference_line.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

class PathOptimizerFileTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    LoadPlaningConfig();
    
    std::string dir = getenv("HOME");
    dir += "/path_optimizer_log/path_data2023-08-31-162827_.csv";
    std::ifstream csv_data(dir.c_str(), std::ios::in);

    std::string line;
    std::getline(csv_data, line);
    std::getline(csv_data, line);
    std::getline(csv_data, line);

    std::stringstream ss(line);
    std::string str;

    std::getline(ss, str, ',');
    start_point_.set_x(std::stod(str));
    std::getline(ss, str, ',');
    start_point_.set_y(std::stod(str));
    std::getline(ss, str, ',');
    start_point_.set_theta(std::stod(str));
    std::getline(ss, str, ',');
    start_point_.set_kappa(std::stod(str));

    std::getline(ss, str, ',');
    init_v_ = std::stod(str);
    std::getline(ss, str, ',');
    target_v_ = std::stod(str);

    std::getline(csv_data, line);
    std::getline(csv_data, line);
    std::vector<pnc::PathPoint> ref_points;
    while (std::getline(csv_data, line)) {
      if (line == "boundary") {
        break;
      }
      pnc::PathPoint point;
      std::stringstream ref_ss(line);
      std::getline(ref_ss, str, ',');
      point.set_x(std::stod(str));
      std::getline(ref_ss, str, ',');
      point.set_y(std::stod(str));
      std::getline(ref_ss, str, ',');
      point.set_theta(std::stod(str));
      std::getline(ref_ss, str, ',');
      point.set_kappa(std::stod(str));
      // point.set_kappa(0.0);
      std::getline(ref_ss, str, ',');
      point.set_s(std::stod(str));

      ref_points.push_back(point);
    }
    reference_line_.set_path_points(ref_points);

    std::getline(csv_data, line);
    std::getline(csv_data, line);
    std::stringstream init_s_ss(line);
    std::getline(init_s_ss, str, ',');
    start_s_ = std::stod(str);
    std::getline(init_s_ss, str, ',');
    delta_s_ = std::stod(str);
    
    std::getline(csv_data, line);
    lane_boundary_.clear();
    while (std::getline(csv_data, line)) {
      if (line == "obstacle boundary") {
        break;
      }
      std::stringstream lane_bound_ss(line);
      std::getline(lane_bound_ss, str, ',');
      double lower = std::stod(str);
      std::getline(lane_bound_ss, str, ',');
      double upper = std::stod(str);
      lane_boundary_.emplace_back(lower, upper);
    }

    obstacle_boundary_.clear();
    while (std::getline(csv_data, line)) {
      if (line == "path boundary") {
        break;
      }
      std::stringstream obs_bound_ss(line);
      std::getline(obs_bound_ss, str, ',');
      double lower = std::stod(str);
      std::getline(obs_bound_ss, str, ',');
      double upper = std::stod(str);
      obstacle_boundary_.emplace_back(lower, upper);
    }

    path_boundary_.clear();
    while (std::getline(csv_data, line)) {
      std::stringstream path_bound_ss(line);
      std::getline(path_bound_ss, str, ',');
      double lower = std::stod(str);
      std::getline(path_bound_ss, str, ',');
      double upper = std::stod(str);
      path_boundary_.emplace_back(lower, upper);
    }

    boundary_.set_start_s(start_s_);
    boundary_.set_delta_s(delta_s_);
    boundary_.set_lane_boundary(lane_boundary_);
    boundary_.set_obstacle_boundary(obstacle_boundary_);
    boundary_.set_path_boundary(path_boundary_);
  }

  void LoadPlaningConfig() {
    std::string planning_config_file = "/home/ws/src/pnc/configs/planning/planning.pb.txt";
    std::string gflag_config_file_path = "/home/ws/src/pnc/configs/planning/planning.conf";
    pnc::File::GetGflagConfig(gflag_config_file_path);
    pnc::File::GetProtoConfig(planning_config_file, &config_);
    pnc::FLAGS_vehicle_config_file = "/home/ws/src/pnc/configs/common/vehicle_config.pb.txt";
    for (const auto& default_task_config : config_.default_task_config()) {
      if (default_task_config.task_type() == pnc::TaskType::PATH_OPTIMIZER) {
        task_config_ = default_task_config;
        break;
      }
    }
  }

 protected:
  double init_v_;
  double target_v_;
  double start_s_;
  double delta_s_;
  pnc::PathPoint start_point_;
  planning::PlanningConfig config_;
  pnc::TaskConfig task_config_;
  VehicleModelOptimizer vehicle_model_optimizer_;
  // VehicleModelOptimizerOSQP vehicle_model_optimizer_;
  pnc_map::ReferenceLine reference_line_;
  std::vector<std::pair<double, double>> lane_boundary_;
  std::vector<std::pair<double, double>> obstacle_boundary_;
  std::vector<std::pair<double, double>> path_boundary_;
  PathBoundary boundary_;
};

TEST_F(PathOptimizerFileTest, Process) {
  vehicle_model_optimizer_.Init();
  vehicle_model_optimizer_.set_weights(
        task_config_.path_optimizer_config().defalut_weights());
  vehicle_model_optimizer_.SetCarSpeedInfo(init_v_, target_v_);
  bool status = vehicle_model_optimizer_.OptimizePath(
      reference_line_, start_point_, &boundary_);
  if (!status) {
    AERROR << "Vehicle Model Optimizer Failed.";
  }

  PathData path_data;
  status = vehicle_model_optimizer_.GetOptimizePath(&path_data);
}

} // namespace planning
} // namespace xju