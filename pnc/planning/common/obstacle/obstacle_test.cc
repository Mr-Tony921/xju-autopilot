/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/obstacle/obstacle.h"

#include <cmath>

#include "gtest/gtest.h"
#include "chassis.pb.h"
#include "prediction_obstacle.pb.h"
#include "pad.pb.h"
#include "planning_config.pb.h"
#include "planning/common/frame/frame.h"
#include "planning/common/indexed_list.h"
#include "planning/common/local_view.h"
#include "common/file/file.h"
#include "common/time/time.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "pnc_map/reference_line.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

using xju::pnc::PerceptionObstacle;

class ObstacleTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    LoadPlaningConfig();
    LoadInput();
    double current_timestamp = 1173545122.22;
    bool status = pnc::VehicleStateProvider::Update(
        *(local_view_.localization), *(local_view_.chassis), 0.0, true);
    pnc::VehicleState vehicle_state = pnc::VehicleStateProvider::GetVehicleState();
    auto reference_lines = GenerateReferenceLines();
    pnc::TrajectoryPoint planning_start_point = PlanningStartPoint();
    frame_.Init(current_timestamp, planning_start_point, vehicle_state, reference_lines, local_view_);
  }

  // generare three streight reference line, y = x
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
    // test
    auto obstacles = Obstacle::CreateObstacles(prediction_obstacles_);
    ASSERT_EQ(2, obstacles.size());
    for (auto& obstacle : obstacles) {
      const auto id = obstacle->id();
      indexed_obstacles_.Add(id, *obstacle);
    }

    AINFO << "mass = " << chassis_.mass();
    AINFO << "obstacle num is " << prediction_obstacles_.prediction_obstacle_size();
  }

 protected:
  planning::PlanningConfig config_;

  pnc::PredictionObstacles prediction_obstacles_;
  pnc::Chassis chassis_;
  pnc::Localization localization_;
  pnc::PadMessage pad_message_;

  planning::Frame frame_;
  planning::LocalView local_view_;
  IndexedObstacles indexed_obstacles_;
};

TEST(Obstacle, IsValidPerceptionObstacle) {
  PerceptionObstacle perception_obstacle;
  EXPECT_FALSE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));

  perception_obstacle.mutable_position()->set_x(2.5);
  perception_obstacle.mutable_position()->set_y(0.5);
  perception_obstacle.mutable_position()->set_z(0.5);

  perception_obstacle.mutable_velocity()->set_x(2.5);
  perception_obstacle.mutable_velocity()->set_y(0.5);
  perception_obstacle.mutable_velocity()->set_z(0.5);
  EXPECT_FALSE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));

  perception_obstacle.set_length(0.1);
  EXPECT_FALSE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));

  perception_obstacle.set_width(0.1);
  EXPECT_FALSE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));

  perception_obstacle.set_height(0.1);
  EXPECT_TRUE(Obstacle::IsValidPerceptionObstacle(perception_obstacle));
}

TEST_F(ObstacleTest, CreateObstacles) {
  ASSERT_EQ(2, indexed_obstacles_.Items().size());
  EXPECT_TRUE(indexed_obstacles_.Find("1"));
  EXPECT_FALSE(indexed_obstacles_.Find("2"));
  EXPECT_TRUE(indexed_obstacles_.Find("2_0"));
}

TEST_F(ObstacleTest, id) {
  const auto* obstacle = indexed_obstacles_.Find("2_0");
  ASSERT_TRUE(obstacle);
  EXPECT_EQ("2_0", obstacle->id());
  EXPECT_EQ(2, obstacle->perception_id());
}

TEST_F(ObstacleTest, GetPointAtTime) {
  const auto* obstacle = indexed_obstacles_.Find("2_0");
  ASSERT_TRUE(obstacle);
  // first
  const auto first_point = obstacle->GetPointAtTime(0.0);
  EXPECT_DOUBLE_EQ(0.0, first_point.relative_time());
  EXPECT_DOUBLE_EQ(4.0, first_point.path_point().x());
  EXPECT_DOUBLE_EQ(30.0, first_point.path_point().y());

  // last
  const auto last_point = obstacle->GetPointAtTime(8.0);
  EXPECT_DOUBLE_EQ(8.0, last_point.relative_time());
  EXPECT_DOUBLE_EQ(4.0, last_point.path_point().x());
  EXPECT_DOUBLE_EQ(70.0, last_point.path_point().y());

  // middle
  const auto middle_point = obstacle->GetPointAtTime(4.5);
  EXPECT_GE(5.0, middle_point.relative_time());
  EXPECT_LE(4.0, middle_point.relative_time());
  EXPECT_GE(4.0, middle_point.path_point().x());
  EXPECT_LE(4.0, middle_point.path_point().x());
  EXPECT_GE(55.0, middle_point.path_point().y());
  EXPECT_LE(50.0, middle_point.path_point().y());
}

TEST_F(ObstacleTest, GetBoundingBox) {
  const auto* obstacle = indexed_obstacles_.Find("2_0");
  ASSERT_TRUE(obstacle);
  const auto& point = obstacle->trajectory().trajectory_point(0);
  const auto& box = obstacle->GetBoundingBox(point);
  std::vector<pnc::Vec2d> corners;
  box.GetAllCorners(&corners);
  EXPECT_EQ(4, corners.size());
  EXPECT_DOUBLE_EQ(4.0, box.length());
  EXPECT_DOUBLE_EQ(2.0, box.width());
  EXPECT_DOUBLE_EQ(4.0, box.center_x());
  EXPECT_DOUBLE_EQ(30.0, box.center_y());
  EXPECT_DOUBLE_EQ(1.57, box.heading());
}

TEST_F(ObstacleTest, polygon) {
  const auto* obstacle = indexed_obstacles_.Find("2_0");
  ASSERT_TRUE(obstacle);
  const auto& polygon = obstacle->polygon();

  const auto& points = polygon.corners();
  EXPECT_EQ(4, points.size());
  EXPECT_DOUBLE_EQ(2.0, points[0].x());
  EXPECT_DOUBLE_EQ(32.0, points[0].y());
  EXPECT_DOUBLE_EQ(2.0, points[1].x());
  EXPECT_DOUBLE_EQ(28.0, points[1].y());
  EXPECT_DOUBLE_EQ(6.0, points[2].x());
  EXPECT_DOUBLE_EQ(28.0, points[2].y());
  EXPECT_DOUBLE_EQ(6.0, points[3].x());
  EXPECT_DOUBLE_EQ(32.0, points[3].y());
}

} // namespace test
} // namespace xju