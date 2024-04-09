/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/interpolation_1d.h"

#include "common/file/file.h"
#include "common/logger/logger.h"
#include "gtest/gtest.h"
#include "scenario_config.pb.h"

namespace xju {
namespace pnc {
class Interpolation1DTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string test_config_file =
        "/home/ws/src/pnc/control/tasks/"
        "mpc_optimizer_test_data/test_scenario_config.pb.txt";
    pnc::ScenarioConfig scenario_config;
    pnc::File::GetProtoConfig(test_config_file, &scenario_config);
    config_ = scenario_config.stage_config()[0].task_config()[2];
  }

 protected:
  pnc::TaskConfig config_;
};

TEST_F(Interpolation1DTest, normal) {
  pnc::Interpolation1D::DataType xy{{0, 0}, {15, 12}, {30, 17}};
  pnc::Interpolation1D table;
  EXPECT_EQ(table.Init(xy), true);

  constexpr double kepsilon = 1.0e-10;
  for (size_t i = 0; i < xy.size(); i++) {
    EXPECT_NEAR(xy[i].second, table.Interpolate(xy[i].first), kepsilon);
  }

  EXPECT_DOUBLE_EQ(table.Interpolate(5), 4.77777777777777777777777);
  EXPECT_DOUBLE_EQ(table.Interpolate(10), 8.7777777777777777777777);
  EXPECT_DOUBLE_EQ(table.Interpolate(20), 14.444444444444444444444);

  // out of x range
  EXPECT_DOUBLE_EQ(table.Interpolate(-1), 0);
  EXPECT_DOUBLE_EQ(table.Interpolate(35), 17);
}

TEST_F(Interpolation1DTest, unordered) {
  pnc::Interpolation1D::DataType xy{{15, 12}, {5, 5}, {40, 25}, {30, 17}};
  pnc::Interpolation1D table;
  EXPECT_TRUE(table.Init(xy));

  constexpr double kepsilon = 1.0e-10;
  for (size_t i = 0; i < xy.size(); i++) {
    EXPECT_NEAR(xy[i].second, table.Interpolate(xy[i].first), kepsilon);
  }
}

TEST_F(Interpolation1DTest, steer_gain_scheduler) {
  const auto& gain_scheduler =
      config_.steer_angle_optimizer_config().steer_gain_scheduler();
  AINFO << "steer_gain_scheduler config: " << gain_scheduler.DebugString();
  Interpolation1D::DataType xy;
  for (const auto& scheduler : gain_scheduler.scheduler()) {
    xy.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  Interpolation1D table;
  EXPECT_TRUE(table.Init(xy));
  for (size_t i = 0; i < xy.size(); i++) {
    EXPECT_DOUBLE_EQ(xy[i].second, table.Interpolate(xy[i].first));
  }
}

}  // namespace pnc
}  // namespace xju
