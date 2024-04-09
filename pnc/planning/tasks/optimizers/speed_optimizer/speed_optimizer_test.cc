/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/speed_optimizer/speed_optimizer.h"

#include <memory>

#include "common/file/file.h"
#include "common/logger/logger.h"
#include "gtest/gtest.h"
#include "planning/common/frame/frame.h"
#include "planning/common/planning_internal/planning_internal.h"
#include "planning/common/reference_line_info/reference_line_info.h"
#include "planning_config.pb.h"
#include "pnc_point.pb.h"

namespace xju {
namespace planning {

class SpeedOptimizerTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    LoadConfig();
    GenerateReferenceLineInfo();
  }

  void LoadConfig() {
    std::string planning_config_file =
        "/home/ws/src/pnc/configs/planning/planning.pb.txt";
    std::string gflag_config_file_path =
        "/home/ws/src/pnc/configs/planning/planning.conf";
    if (!pnc::File::GetGflagConfig(gflag_config_file_path)) {
      AERROR << "load planning gflag failed";
      return;
    }
    if (!pnc::File::GetProtoConfig(planning_config_file, &config_)) {
      AERROR << "load proto config failed";
      return;
    }
    for (const auto& default_task_config : config_.default_task_config()) {
      if (default_task_config.task_type() == pnc::TaskType::SPEED_OPTIMIZER) {
        task_config_ = default_task_config;
        break;
      }
    }
  }

  void GenerateReferenceLineInfo() {
    int num_of_knots = 100;
    // path_data
    auto path_data = ref_line_info_.mutable_path_data();
    std::vector<pnc::PathPoint> path_points;
    for (int i = 0; i < num_of_knots; i++) {
      pnc::PathPoint path_point;
      path_point.set_x(i * 2.0);
      path_point.set_y(0.0);
      path_point.set_z(0.0);
      path_point.set_theta(0.0);
      path_point.set_kappa(0.0);
      path_point.set_dkappa(0.0);
      path_point.set_ddkappa(0.0);
      path_point.set_s(i * 2.0);
      path_points.push_back(path_point);
    }
    path_data->set_reference_points(path_points);

    std::vector<pnc::FrenetFramePoint> frenet_frame_points;
    for (int i = 0; i < num_of_knots; i++) {
      pnc::FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(i * 1.7);
      frenet_frame_point.set_l(0.0);
      frenet_frame_point.set_ds(10.0);
      frenet_frame_point.set_dds(1.0);
      frenet_frame_point.set_dl(0.0);
      frenet_frame_point.set_ddl(0.0);
      frenet_frame_point.set_kappa(0.0);
      frenet_frame_point.set_dkappa(0.0);
      frenet_frame_point.set_heading_error(0.0);
      frenet_frame_point.set_heading_error(0.0);
      frenet_frame_points.push_back(frenet_frame_point);
    }
    path_data->set_frenet_points(frenet_frame_points);

    // speed_data
    auto speed_data = ref_line_info_.mutable_speed_data();
    speed_data->clear();
    std::array<double, 3> s_init = {0.0, 10.0, 1.0};
    for (int i = 0; i <= 90; i++) {
      double t = 0.1 * i;
      double s = s_init[0] + s_init[1] * t + 0.5 * s_init[2] * t * t;
      double v = s_init[1] + s_init[2] * t;
      double a = s_init[2];
      double da = 0.0;
      speed_data->AppendSpeedPoint(t, s, v, a, da);
      // AINFO <<"t = "<<t<<"  s= "<<s<<"  v= "<<v<<"  a= "<<a;
    }

    // st_graph_data
    auto st_graph_data = ref_line_info_.mutable_st_graph_data();
    std::vector<const STBoundary*> st_boundaries;
    pnc::TrajectoryPoint init_point;
    SpeedLimit speed_limit;
    std::vector<std::string> obstacles_id_set;
    double cruise_speed;
    double path_data_length;
    //(1)init_point
    init_point.set_a(1.0);
    init_point.set_v(10.0);

    //(2)path_length
    path_data_length = 180.0;

    //(3)st_boundaries
    // TODO

    //(4)speed_limit
    for (int i = 0; i < num_of_knots; i++) {
      speed_limit.AppendSpeedLimit(i * 2, 50.0);
    }

    st_graph_data->LoadData(st_boundaries, init_point, speed_limit,
                            obstacles_id_set, cruise_speed, path_data_length);
  }

 protected:
  ReferenceLineInfo ref_line_info_;
  PlanningConfig config_;
  pnc::TaskConfig task_config_;
};

TEST_F(SpeedOptimizerTest, Optimize) {
  Frame frame_;
  std::shared_ptr<PlanningInternal> internal_ = nullptr;

  SpeedOptimizer speed_optimizer(task_config_, internal_);
  bool res = speed_optimizer.Process(&ref_line_info_, &frame_);
  EXPECT_TRUE(res);
}

}  // namespace planning
}  // namespace xju