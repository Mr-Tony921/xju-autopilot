/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/path_optimizer/vehicle_model_optimizer.h"

#include "common/math/math_utils.h"
#include "gtest/gtest.h"
#include "chassis.pb.h"
#include "prediction_obstacle.pb.h"
#include "planning_config.pb.h"
#include "common/file/file.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "pnc_map/reference_line.h"
#include "common/logger/logger.h"
#include "common/time/time.h"

namespace xju {
namespace planning {

class VehicleModelOptimizerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    LoadPlaningConfig();
    LoadWeights();
    reference_line_ = GenerateReferenceLine();
    planning_start_point_ = PlanningStartPoint();
    path_boundary_ = GenerateBoundary();
  }

  // generare three streight reference line, y = x
  pnc_map::ReferenceLine GenerateReferenceLine() {
    pnc_map::ReferenceLine ref_line;
    std::vector<pnc::PathPoint> points;

    for (int i = 0; i < 500; ++i) {
      pnc::PathPoint pt;
      pt.set_x(i / 2.0);
      pt.set_y(i / 2.0);
      pt.set_z(0.0);
      pt.set_theta(M_PI_2 / 2.0);
      pt.set_kappa(0.0);
      pt.set_dkappa(0.0);
      pt.set_ddkappa(0.0);
      pt.set_s(i / 2.0 * std::sqrt(2.0));
      points.push_back(pt);
    }

    ref_line.set_path_points(points);
    ref_line.set_lane_width(3.75 / 2.0);
    ref_line.set_min_speed_limit(0.0);
    ref_line.set_max_speed_limit(120.0 / 3.6);
    ref_line.set_is_emergency_lane(false);
    ref_line.set_is_death_lane(false);
    ref_line.set_is_recommended_lane(false);

    ref_line.set_id(std::to_string(0));
    ref_line.set_left_lane_id(std::to_string(1));
    ref_line.set_right_lane_id(std::to_string(-1));
    ref_line.set_lane_order(pnc::LaneOrder::SECOND_LANE);

    return ref_line;
  }

  pnc::TrajectoryPoint PlanningStartPoint() {
    pnc::TrajectoryPoint tp;
    auto path_point = tp.mutable_path_point();
    path_point->set_x(30.0);
    path_point->set_y(30.0);
    path_point->set_z(0.0);
    path_point->set_theta(M_PI_2 / 2.0);
    path_point->set_kappa(0.0);
    path_point->set_dkappa(0.0);
    path_point->set_ddkappa(0.0);
    path_point->set_s(0.0);

    tp.set_v(0.0);
    tp.set_a(0.0);
    tp.set_da(0.0);
    tp.set_relative_time(0.0);
    return tp;
  }

  void LoadPlaningConfig() {
    std::string planning_config_file = "/home/ws/src/pnc/configs/planning/planning.pb.txt";
    std::string gflag_config_file_path = "/home/ws/src/pnc/configs/planning/planning.conf";
    pnc::File::GetGflagConfig(gflag_config_file_path);
    pnc::File::GetProtoConfig(planning_config_file, &config_);
    pnc::VehicleStateProvider::Init(config_.vehicle_state_config());
  }

  void LoadWeights() {
    for (const auto& default_task_config : config_.default_task_config()) {
      if (default_task_config.task_type() == pnc::TaskType::PATH_OPTIMIZER) {
        weights_ = default_task_config.path_optimizer_config().defalut_weights();
        return;
      }
    }
  }

  PathBoundary GenerateBoundary() {
    double start_s = 0.0;
    double delta_s = 0.5;

    std::vector<std::pair<double, double>> lane_boundary;
    std::vector<std::pair<double, double>> obstacle_boundary;

    for (int i = 0; i < reference_line_.reference_points().size(); ++i) {
      // lane_boundary.emplace_back(-1.75, 1.75);
      // obstacle_boundary.emplace_back(-5, 5);

      lane_boundary.emplace_back(-1.75 + 1.25, 3.0 - 1.25);
      if (i < 150) {
        obstacle_boundary.emplace_back(-5 + 1.25, 5 - 1.25);
      } else {
        obstacle_boundary.emplace_back(-1.0 + 1.25, 5 - 1.25);
      }
      
    }
    PathBoundary path_boundary = PathBoundary(start_s, delta_s, lane_boundary, obstacle_boundary);
    return path_boundary;
  }

 protected:
  PlanningConfig config_;
  PathOptimizerWeights weights_;
  pnc_map::ReferenceLine reference_line_;
  pnc::TrajectoryPoint planning_start_point_;
  PathBoundary path_boundary_;

  VehicleModelOptimizer vehicle_model_ocp_;
};

TEST_F(VehicleModelOptimizerTest, OptimizePath) {
  double now = pnc::Time::NowInSeconds();
  vehicle_model_ocp_.Init();
  vehicle_model_ocp_.set_weights(weights_);
  bool status = vehicle_model_ocp_.OptimizePath(
      reference_line_, planning_start_point_.path_point(), &path_boundary_);
  EXPECT_TRUE(status);
  PathData path_data;
  if (status) {
    vehicle_model_ocp_.GetOptimizePath(&path_data);
  }
  AINFO << "time cost = " << pnc::Time::NowInSeconds() - now;
}

} // namespace planning
} // namespace xju