/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/deciders/speed_decider/trajectory1d_generator.h"

#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "vehicle_config.pb.h"
#include "planning_config.pb.h"
#include "planning_task_config.pb.h"
#include "common/file/file.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "common/math/polynomial_x_order.h"
#include "planning/common/speed/st_graph_data.h"
#include "planning/common/speed/st_boundary.h"
#include "planning_config.pb.h"
#include "planning/tasks/deciders/speed_decider/feasible_region.h"


namespace xju {
namespace planning {

pnc::TrajectoryPoint PlanningStartPoint() {
  pnc::TrajectoryPoint tp;
  auto path_point = tp.mutable_path_point();
  path_point->set_x(0.0);
  path_point->set_y(10.0);
  path_point->set_z(0.0);
  path_point->set_theta(M_PI_2);
  path_point->set_kappa(0.0);
  path_point->set_dkappa(0.0);
  path_point->set_ddkappa(0.0);
  path_point->set_s(0.0);

  tp.set_v(5.0);
  tp.set_a(0.0);
  tp.set_da(0.0);
  tp.set_relative_time(0.0);
  return tp;
}

class Trajectory1dGeneratorTest : public testing::Test {
 public:
  virtual void SetUp() {
    LoadPlanningConfig();
    LoadTaskConfig();
    GenerateStGraphData();
    SpeedDeciderConfig speed_decider_config = task_config_.speed_decider_config();
    auto vehicle_config = pnc::VehicleConfigProvider::GetConfig();
    std::array<double, 3> init_s{0.0, 5.0, 0.0};
    double cruising_speed = 33.0;
    trajectory1d_generator_ = new Trajectory1dGenerator(
        cruising_speed, speed_decider_config, vehicle_config, init_s, st_graph_data_);
  }
  virtual void TearDown() {
    delete trajectory1d_generator_;
  }
  void LoadPlanningConfig() {
    std::string planning_config_file = "/home/ws/src/pnc/configs/planning/planning.pb.txt";
    std::string gflag_config_file_path = "/home/ws/src/pnc/configs/planning/planning.conf";
    pnc::File::GetGflagConfig(gflag_config_file_path);
    pnc::File::GetProtoConfig(planning_config_file, &config_);
    pnc::VehicleStateProvider::Init(config_.vehicle_state_config());
  }
  void LoadTaskConfig() {
    for (const auto& task_config : config_.default_task_config()) {
      if (task_config.task_type() == pnc::TaskType::SPEED_DECIDER) {
        task_config_ = task_config;
        break;
      }
    }
    return;
  }

  void GenerateStGraphData() {
    std::vector<const STBoundary*> st_boundaries;
    std::vector<std::string> obstacles_id_set;
    std::vector<STPoint> upper_st_points;
    std::vector<STPoint> lower_st_points;
    lower_st_points.emplace_back(80, 0.0);
    lower_st_points.emplace_back(80, 8.0);
    upper_st_points.emplace_back(85, 0.0);
    upper_st_points.emplace_back(85, 8.0);
    auto st_boundary_1 = new STBoundary(STBoundary::CreateInstance(
        lower_st_points, upper_st_points));
    // st_boundary_1->set_boundary_type(STBoundary::BoundaryType::STOP);

    st_boundaries.push_back(st_boundary_1);

    obstacles_id_set.push_back("1");
    lower_st_points.clear();
    upper_st_points.clear();
      
    lower_st_points.emplace_back(35, 0.0);
    lower_st_points.emplace_back(70, 8.0);
    upper_st_points.emplace_back(40, 0.0);
    upper_st_points.emplace_back(75, 8.0);
    auto st_boundary_2 = new STBoundary(STBoundary::CreateInstance(
        lower_st_points, upper_st_points));
    st_boundaries.push_back(st_boundary_2);
    obstacles_id_set.push_back("2_0");

    SpeedLimit speed_limit;
    for(int s = 0; s <= 100; s += 5) {
      speed_limit.AppendSpeedLimit(s, 120 / 3.6);
    }
    auto init_point = PlanningStartPoint();
    st_graph_data_.LoadData(st_boundaries, init_point,
      speed_limit, obstacles_id_set, 90 / 3.6, 100);
  }
  planning::PlanningConfig config_;
  pnc::TaskConfig task_config_;
  StGraphData st_graph_data_;
  Trajectory1dGenerator* trajectory1d_generator_;
}; 


// TEST_F(Trajectory1dGeneratorTest, CalculateQuarticPolyCurve) {

//   std::shared_ptr<pnc::PolynomialXOrder> s_poly = new pnc::PolynomialXOrder();
//   trajectory1d_generator_->CalculateQuarticPolyCurve(0.0, 0.0, 0.0, 0.0, 0.0, 0.1, s_poly);

// }

// TEST_F(Trajectory1dGeneratorTest, CalculateQuinticPolyCurve) {

//   pnc::PolynomialXOrder s_poly;
//   trajectory1d_generator_->CalculateQuinticPolyCurve(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, &s_poly);
// }

// TEST_F(Trajectory1dGeneratorTest, GenerateSpeedProfilesForCruising) {

//   double cruising_speed = 33;
//   std::vector<std::shared_ptr<pnc::PolynomialXOrder>> lon_trajectory_bundle;
//   lon_trajectory_bundle.clear();
//   bool ret = trajectory1d_generator_->GenerateSpeedProfilesForCruising(
//       cruising_speed, &lon_trajectory_bundle);
//   AERROR << lon_trajectory_bundle.size();

// }

// TEST_F(Trajectory1dGeneratorTest, GenerateSpeedProfilesForStopping) {

//   double cruising_speed = 33;
//   std::vector<std::shared_ptr<pnc::PolynomialXOrder>> lon_trajectory_bundle;
//   lon_trajectory_bundle.clear(); 
//   if (!st_graph_data_.st_boundaries().empty()) {
//     auto stop_st_boundary = st_graph_data_.st_boundaries().front();
//     trajectory1d_generator_->GenerateSpeedProfilesForStopping(
//         *stop_st_boundary, &lon_trajectory_bundle);
//     AERROR << lon_trajectory_bundle.size();
//   }
// }

// TEST_F(Trajectory1dGeneratorTest, GenerateSpeedProfilesForFollow) {

//   double cruising_speed = 33;
//   std::vector<std::shared_ptr<pnc::PolynomialXOrder>> lon_trajectory_bundle;
//   lon_trajectory_bundle.clear(); 
//   if (!st_graph_data_.st_boundaries().empty()) {
//     auto st_boundary = st_graph_data_.st_boundaries().back();
//     trajectory1d_generator_->GenerateSpeedProfilesForFollow(
//         cruising_speed, *st_boundary, &lon_trajectory_bundle);
//     AERROR << lon_trajectory_bundle.size();
//   }
// }

// TEST_F(Trajectory1dGeneratorTest, GenerateSpeedProfilesForOvertake) {

//   double cruising_speed = 33;
//   std::vector<std::shared_ptr<pnc::PolynomialXOrder>> lon_trajectory_bundle;
//   lon_trajectory_bundle.clear(); 
//   if (!st_graph_data_.st_boundaries().empty()) {
//     auto st_boundary = st_graph_data_.st_boundaries().back();
//     trajectory1d_generator_->GenerateSpeedProfilesForOvertake(
//         cruising_speed, *st_boundary, &lon_trajectory_bundle);
//     AERROR << lon_trajectory_bundle.size();
//   }
// }


TEST_F(Trajectory1dGeneratorTest, GenerateLonTrajectoryBundle) {


  std::vector<std::shared_ptr<pnc::PolynomialXOrder>> lon_trajectory_bundle;
  lon_trajectory_bundle.clear(); 
  trajectory1d_generator_->GenerateLonTrajectoryBundle(
      &lon_trajectory_bundle);
  AINFO << lon_trajectory_bundle.size();
}


// TEST_F(Trajectory1dGeneratorTest, ComputeStopGuideVelocity) {

//   double cruising_speed = 33;
//   std::vector<std::shared_ptr<pnc::PolynomialXOrder>> lon_trajectory_bundle;
//   lon_trajectory_bundle.clear(); 
//   trajectory1d_generator_->GenerateLonTrajectoryBundle(
//       cruising_speed, &lon_trajectory_bundle);
//   auto ref_s_dot = trajectory1d_generator_->ComputeGuideVelocity(cruising_speed);
//   AINFO << ref_s_dot.size();
// }

// TEST_F(Trajectory1dGeneratorTest, TrajectoryEvaluate) {
//   std::vector<std::shared_ptr<pnc::PolynomialXOrder>> lon_trajectory_bundle;
//   lon_trajectory_bundle.clear(); 
//   trajectory1d_generator_->GenerateLonTrajectoryBundle(
//       &lon_trajectory_bundle);

//   pnc::PolynomialXOrder final_poly;
//   trajectory1d_generator_->TrajectoryEvaluate(&lon_trajectory_bundle, &final_poly);
//   AINFO << final_poly.end_state().first[0] << " " << final_poly.end_state().first[1]
//         << " t = " <<final_poly.end_state().second;
// }


} // namespace planning
} // namespace xju



