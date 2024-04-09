/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/deciders/path_decider/path_decider.h"

#include "chassis.pb.h"
#include "prediction_obstacle.pb.h"
// #include "pad.pb.h"
#include "planning_config.pb.h"
#include "task_config.pb.h"

#include "common/file/file.h"
#include "common/time/time.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "common/logger/logger.h"

#include "pnc_map/reference_line.h"
#include "gtest/gtest.h"

namespace xju {
namespace planning {

class PathDeciderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    ADEBUG << "SetUp start";
    LoadPlaningConfig();
    LoadTaskConfig();
    LoadInput();
    double current_timestamp = 1173545122.22;
    bool status = pnc::VehicleStateProvider::Update(
        *(local_view_.localization), *(local_view_.chassis_info));
    pnc::VehicleState vehicle_state = pnc::VehicleStateProvider::GetVehicleState();
    auto reference_lines = GenerateReferenceLines();
    pnc::TrajectoryPoint planning_start_point = PlanningStartPoint();
    frame_.Init(current_timestamp, planning_start_point, vehicle_state, reference_lines, local_view_);
    UpdateInternal();
  }

  void UpdateInternal() {
    internal_ = std::make_shared<PlanningInternal>();
  } 

  // generare three streight reference line, y = x
  std::list<pnc_map::ReferenceLine> GenerateReferenceLines() {
    std::list<pnc_map::ReferenceLine> ref_lines;
    for (int k = 0; k < 3; ++k) {
      pnc_map::ReferenceLine ref_line;
      std::vector<pnc::PathPoint> points;
      double x = 0.0;
      double y = 0.0;
      if (k == 0) {
        x = -3.75;
      } else if (k == 2) {
        x = 3.75;
      }
      for (int i = 0; i < 30; ++i) {
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
      ref_line.set_lane_width(3.75);
      ref_line.set_min_speed_limit(0.0);
      ref_line.set_max_speed_limit(120.0 / 3.6);
      ref_line.set_left_lane_marking_type(pnc::LaneMarkingType::LBR_MARKING_SINGLE_DASHED_LINE);
      ref_line.set_right_lane_marking_type(pnc::LaneMarkingType::LBR_MARKING_SINGLE_DASHED_LINE);
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
    double left_width = -1.0;
    double right_width = -1.0;
    ref_line->GetLaneWidth(0.0, left_width, right_width);
    
    ++ref_line;
    ref_line->set_id(std::to_string(1));
    ref_line->set_left_lane_id("");
    ref_line->set_right_lane_id(std::to_string(0));
    ref_line->set_lane_order(pnc::LaneOrder::THIRD_LANE);
    ref_line->set_left_lane_marking_type(pnc::LaneMarkingType::LBR_MARKING_SINGLE_SOLID_LINE);

    ++ref_line;
    ref_line->set_id(std::to_string(-1));
    ref_line->set_left_lane_id(std::to_string(0));
    ref_line->set_right_lane_id("");
    ref_line->set_lane_order(pnc::LaneOrder::FIRST_LANE);
    ref_line->set_right_lane_marking_type(pnc::LaneMarkingType::LBR_MARKING_SINGLE_SOLID_LINE);

    return ref_lines;
  }

  pnc::TrajectoryPoint PlanningStartPoint() {
    pnc::TrajectoryPoint tp;
    auto path_point = tp.mutable_path_point();
    path_point->set_x(0.0);
    path_point->set_y(0.0);
    path_point->set_z(0.0);
    path_point->set_theta(0.0);
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
    ADEBUG << "LoadPlaningConfig start";
    std::string gflag_config_file_path = "/home/ws/src/pnc/configs/planning/planning.conf";
    pnc::File::GetGflagConfig(gflag_config_file_path);
    pnc::File::GetProtoConfig(planning_config_file, &config_);
    pnc::VehicleStateProvider::Init(config_.vehicle_state_config());
  }

  void LoadTaskConfig() {
    for (const auto& default_task_config : config_.default_task_config()) {
      if (default_task_config.task_type()  == pnc::TaskType::PATH_DECIDER) {
        task_config_ = default_task_config;
        break;
      }
    }
  }

  void GeneratePathData(std::shared_ptr<ReferenceLineInfo> reference_line_info) {
  auto* path_data = reference_line_info->mutable_path_data();
  std::vector<pnc::PathPoint> path_points;
  for (int i = 0; i < 30; ++i) {
    pnc::PathPoint pt;
    pt.set_x(0);
    pt.set_y(i / 2.0);
    pt.set_z(0.0);
    pt.set_theta(M_PI_2);
    pt.set_kappa(0.0);
    pt.set_dkappa(0.0);
    pt.set_ddkappa(0.0);
    pt.set_s(i / 2.0);
    path_points.push_back(pt);     
  }
  path_data->set_reference_points(path_points);

  std::vector<pnc::FrenetFramePoint> frenet_points;
  for (int i = 0; i < 30; ++i) {
    pnc::FrenetFramePoint pt;
    pt.set_s(i/2.0);
    pt.set_l(0);
    pt.set_heading_error(0);
    pt.set_kappa(0);
    frenet_points.push_back(pt);
  }
  AINFO << "frenet_points.back().s()" << frenet_points.back().s();
  path_data->set_frenet_points(frenet_points);
  
}

  void LoadInput() {
    std::string prediction_file = "/home/ws/src/pnc/planning/tasks/deciders/path_decider/path_decider_test/prediction_obstacles.pb.txt";
    std::string chassis_file = "//home/ws/src/pnc/planning/tasks/deciders/path_decider/path_decider_test/chassis.pb.txt";
    std::string localization_file = "/home/ws/src/pnc/planning/tasks/deciders/path_decider/path_decider_test/localization.pb.txt";
    std::string pad_file = "/home/ws/src/pnc/planning/tasks/deciders/path_decider/path_decider_test/pad_message.pb.txt";
    pnc::File::GetProtoConfig(prediction_file, &prediction_obstaces_);
    pnc::File::GetProtoConfig(chassis_file, &chassis_);
    pnc::File::GetProtoConfig(localization_file, &localization_);
    // pnc::File::GetProtoConfig(pad_file, &pad_message_);
    ADEBUG << "prediction_obstaces__size() = " << prediction_obstaces_.prediction_obstacle().size();
    local_view_.prediction_obstacles = std::make_shared<pnc::PredictionObstacles>(prediction_obstaces_);
    local_view_.chassis_info = std::make_shared<pnc::ChassisInfo>(chassis_);
    local_view_.localization = std::make_shared<pnc::Localization>(localization_);
    // local_view_.pad_msg = std::make_shared<pnc::PadMessage>(pad_message_);
  }

 protected:
  planning::PlanningConfig config_;
  pnc::TaskConfig task_config_;

  pnc::PredictionObstacles prediction_obstaces_;
  pnc::ChassisInfo chassis_;
  pnc::Localization localization_;
  // pnc::PadMessage pad_message_;

  planning::Frame frame_;
  planning::LocalView local_view_;
  std::shared_ptr<PlanningInternal> internal_;
};

TEST_F(PathDeciderTest, Process) {
  ADEBUG << "PathDeciderTest start";
  auto path_decider_task = std::make_shared<PathDecider>(task_config_, internal_);
  AINFO << "reference_line_size= " << (*frame_.GetReferenceLineInfos()).size();
  auto reference_line_info = frame_.GetReferenceLineInfos();
  path_decider_task->Init(task_config_);
  auto iter = reference_line_info->begin();
  auto reference_line_info_second = *(++iter);
  reference_line_info_second.SetBlockingObstacle("3");
  AINFO << "reference_line_id = " << reference_line_info_second.reference_line().id();
  // FrenetPoints(&reference_line_info_second);
  GeneratePathData(&reference_line_info_second);
  bool status = path_decider_task->Process(&reference_line_info_second, &frame_);
  EXPECT_TRUE(status);
}


} // namespace planning
} // namespace xju
