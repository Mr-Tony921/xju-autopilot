/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/reference_line_info/reference_line_info.h"

#include <cmath>
#include <string>

#include "gtest/gtest.h"
#include "chassis.pb.h"
#include "prediction_obstacle.pb.h"
#include "pad.pb.h"
#include "planning_config.pb.h"
#include "common/file/file.h"
#include "common/time/time.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "common/logger/logger.h"
#include "planning/common/frame/frame.h"
#include "planning/common/local_view.h"


namespace xju {
namespace planning {

std::vector<const Obstacle*> GenerateObstacles (
    const pnc::PredictionObstacles& predictions) {
  std::vector<const Obstacle*> obstacles;
  auto list_obs = Obstacle::CreateObstacles(predictions);
  for(auto& obs : list_obs) {
    Obstacle *ptr_obs= new Obstacle(*obs);
    obstacles.push_back(ptr_obs);
  }
  return obstacles;
}
SpeedData GenerateSpeedData() {
  SpeedData speed_data;
  for(double t = 0.0; t < 8; t += 0.1) {
    double v = 10.0;
    double s = v * (t);
    speed_data.AppendSpeedPoint(t, s, v, 0.0, 0.0);
  }
  return speed_data;
}
PathData GeneratePathData() {
  PathData path_data;
  std::vector<pnc::PathPoint> path_points;
  for (int i = 0; i < 101; ++i) {
    pnc::PathPoint pt;
    pt.set_x(0);
    pt.set_y(i / 2.0 + 30);
    pt.set_z(0.0);
    pt.set_theta(M_PI_2);
    pt.set_kappa(0.0);
    pt.set_dkappa(0.0);
    pt.set_ddkappa(0.0);
    pt.set_s(i / 2.0);
    path_points.push_back(pt);     
  }
  path_data.set_reference_points(path_points);

  std::vector<pnc::FrenetFramePoint> frenet_points;
  for (int i = 0; i < 101; ++i) {
    pnc::FrenetFramePoint pt;
    pt.set_l(0);
    pt.set_heading_error(0);
    pt.set_kappa(0);
    frenet_points.push_back(pt);
  }
  path_data.set_frenet_points(frenet_points);
  return path_data;
}


class ReferenceLineInfoTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    LoadPlaningConfig();
    LoadInput();
    bool status = pnc::VehicleStateProvider::Update(
        *(local_view_.localization), *(local_view_.chassis), 0.0, true);
    pnc::VehicleState vehicle_state = pnc::VehicleStateProvider::GetVehicleState();
    std::list<pnc_map::ReferenceLine> reference_lines = GenerateReferenceLines();
    pnc::TrajectoryPoint planning_start_point = PlanningStartPoint();
    reference_line_info_ = ReferenceLineInfo(vehicle_state, 
        planning_start_point, reference_lines.front());

  }



  std::list<pnc_map::ReferenceLine> GenerateReferenceLines() {
    std::list<pnc_map::ReferenceLine> ref_lines;
    for (int k = 0; k < 3; ++k) {
      pnc_map::ReferenceLine ref_line;
      std::vector<pnc::PathPoint> points;
      double x = 0.0;
      double y = 0.0;
      if (k == 1) {
        x = -3.75;
      } else if (k == 2) {
        x = 3.75;
      }
      for (int i = 0; i < 200; ++i) {
        pnc::PathPoint pt;
        pt.set_x(x);
        pt.set_y(i / 2.0);
        pt.set_z(0.0);
        pt.set_theta(M_PI_2);
        pt.set_kappa(0.0);
        pt.set_dkappa(0.0);
        pt.set_ddkappa(0.0);
        pt.set_s(i / 2.0);
        points.push_back(pt);
      }

      ref_line.set_path_points(points);
      ref_line.set_lane_width(3.75 / 2.0);
      ref_line.set_min_speed_limit(0.0);
      ref_line.set_max_speed_limit(120.0 / 3.6);
      ref_line.set_left_lane_marking_type(pnc::LaneMarkingType::SINGLE_WHITE_DASHED_LINE);
      ref_line.set_right_lane_marking_type(pnc::LaneMarkingType::SINGLE_WHITE_DASHED_LINE);
      ref_line.set_accommodation_lane_type(pnc::AccommodationLaneType::NONE_ACCOMMODATION_LANE_TYPE);
      ref_line.set_is_emergency_lane(false);
      ref_line.set_is_death_lane(false);
      ref_line.set_is_recommended_lane(false);
      ref_lines.push_back(ref_line);
    }

    auto ref_line = ref_lines.begin();
    ref_line->set_id(std::to_string(0));
    ref_line->set_left_lane_id(std::to_string(1));
    ref_line->set_right_lane_id(std::to_string(-1));
    ref_line->set_lane_order(pnc::LaneOrder::SECOND_LANE);

    ++ref_line;
    ref_line->set_id(std::to_string(1));
    ref_line->set_left_lane_id("");
    ref_line->set_right_lane_id(std::to_string(0));
    ref_line->set_lane_order(pnc::LaneOrder::THIRD_LANE);
    ref_line->set_left_lane_marking_type(pnc::LaneMarkingType::SINGLE_WHITE_SOLID_LINE);

    ++ref_line;
    ref_line->set_id(std::to_string(-1));
    ref_line->set_left_lane_id(std::to_string(0));
    ref_line->set_right_lane_id("");
    ref_line->set_lane_order(pnc::LaneOrder::FIRST_LANE);
    ref_line->set_right_lane_marking_type(pnc::LaneMarkingType::SINGLE_WHITE_SOLID_LINE);

    return ref_lines;
  }
  
  pnc::TrajectoryPoint PlanningStartPoint() {
    pnc::TrajectoryPoint tp;
    auto path_point = tp.mutable_path_point();
    path_point->set_x(0.0);
    path_point->set_y(30.0);
    path_point->set_z(0.0);
    path_point->set_theta(M_PI_2);
    path_point->set_kappa(0.0);
    path_point->set_dkappa(0.0);
    path_point->set_ddkappa(0.0);
    path_point->set_s(0.0);

    tp.set_v(0.0);
    tp.set_a(0.0);
    tp.set_da(0.0);
    tp.set_relative_time(0.1);
    return tp;
  }


  void LoadPlaningConfig() {
    std::string planning_config_file = "/home/ws/src/pnc/configs/planning/planning.pb.txt";
    std::string gflag_config_file_path = "/home/ws/src/pnc/configs/planning/planning.conf";
    pnc::File::GetGflagConfig(gflag_config_file_path);
    pnc::File::GetProtoConfig(planning_config_file, &config_);
    pnc::VehicleStateProvider::Init(config_.vehicle_state_config());
    return;
  }

  void LoadInput() {
    std::string prediction_file = "/home/ws/src/pnc/test_data/prediction_obstacles.pb.txt";
    std::string chassis_file = "/home/ws/src/pnc/test_data/chassis.pb.txt";
    std::string localization_file = "/home/ws/src/pnc/test_data/localization.pb.txt";
    std::string pad_file = "/home/ws/src/pnc/test_data/pad_message.pb.txt";
    pnc::File::GetProtoConfig(prediction_file, &prediction_obstacles_);
    pnc::File::GetProtoConfig(chassis_file, &chassis_);
    pnc::File::GetProtoConfig(localization_file, &localization_);
    pnc::File::GetProtoConfig(pad_file, &pad_message_);

    local_view_.prediction_obstacles = std::make_shared<pnc::PredictionObstacles>(prediction_obstacles_);
    local_view_.chassis = std::make_shared<pnc::Chassis>(chassis_);
    local_view_.localization = std::make_shared<pnc::Localization>(localization_);
    local_view_.pad_msg = std::make_shared<pnc::PadMessage>(pad_message_);
  
  
    AINFO << "mass = " << chassis_.mass();
    AINFO << "obstacle num is " << prediction_obstacles_.prediction_obstacle_size();
    // auto obstacles = Obstacle::CreateObstacles(prediction_obstacles_);
    // AINFO << "111111";
    // for (auto& obstacle : obstacles) {
    //   const Obstacle* ptr_obs = obstacle.get();
    //   AINFO << "obstacle id: " << ptr_obs->id();
    //   obstacles_.push_back(ptr_obs);
    // }
    
    // for(int i = 0; i < obstacles_.size(); ++i) {
    //   AINFO << "obs is static : " << obstacles_.at(i)->id();
    // }
    return;
  }
 
  
 protected:
  ReferenceLineInfo reference_line_info_;
  planning::PlanningConfig config_;

  pnc::PredictionObstacles prediction_obstacles_;
  pnc::Chassis chassis_;
  pnc::Localization localization_;
  pnc::PadMessage pad_message_;

  planning::Frame frame_;
  planning::LocalView local_view_;

};

TEST_F(ReferenceLineInfoTest, Init) {
  auto obstacles = GenerateObstacles(prediction_obstacles_);
  bool ret = reference_line_info_.Init(obstacles);
  EXPECT_TRUE(ret);
}

TEST_F(ReferenceLineInfoTest, AddObstacles) {
  auto obstacles = GenerateObstacles(prediction_obstacles_);
  AERROR << obstacles[1]->id();
  AERROR << obstacles[0]->id();
  bool ret = reference_line_info_.AddObstacles(obstacles);
  EXPECT_TRUE(ret);
}

TEST_F(ReferenceLineInfoTest, IsIrrelevantObstacle) {
  auto obstacles = GenerateObstacles(prediction_obstacles_);
  bool ret_1 = reference_line_info_.IsIrrelevantObstacle(*obstacles[0]);
  EXPECT_FALSE(ret_1);
  bool ret_2 = reference_line_info_.IsIrrelevantObstacle(*obstacles[1]);
  EXPECT_FALSE(ret_2);
}
TEST_F(ReferenceLineInfoTest, CombinePathAndSpeedProfile) {
  auto path_data = GeneratePathData();
  auto* mutable_path_data = reference_line_info_.mutable_path_data();
  *mutable_path_data = path_data;

  auto speed_data = GenerateSpeedData();
  auto* mutable_speed_data = reference_line_info_.mutable_speed_data();
  *mutable_speed_data = speed_data;
  std::vector<pnc::TrajectoryPoint> traj_points;
  traj_points.push_back(reference_line_info_.planning_start_point());

  DiscretizedTrajectory discretized_trajectory(traj_points);
  discretized_trajectory.Clear();
  bool ret = reference_line_info_.CombinePathAndSpeedProfile(&discretized_trajectory);
  EXPECT_TRUE(ret);

}


} // namespace planning
} // namespace xju

