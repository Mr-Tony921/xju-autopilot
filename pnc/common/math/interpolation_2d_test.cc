/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/interpolation_2d.h"

#include "common/file/file.h"
#include "common/logger/logger.h"
#include "gtest/gtest.h"
#include "scenario_config.pb.h"
#include "scenario_config.pb.h"

namespace xju {
namespace pnc {

class Interpolation2DTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    std::string control_conf_file =
        "/home/ws/src/pnc/control/tasks/"
        "mpc_optimizer_test_data/test_scenario_config.pb.txt";
    pnc::ScenarioConfig scenario_config;
    pnc::File::GetProtoConfig(test_config_file, &scenario_config);
    config_ = scenario_config.stage_config()[0].task_config()[1];
  }

 protected:
  ControlConf control_conf_;
};

TEST_F(Interpolation2DTest, normal) {
  pnc::Interpolation2D::DataType xyz{std::make_tuple(0.3, 0.2, 0.6),
                                std::make_tuple(10.1, 15.2, 5.5),
                                std::make_tuple(20.2, 10.3, 30.5)};

  pnc::Interpolation2D estimator;
  EXPECT_TRUE(estimator.Init(xyz));

  for (unsigned i = 0; i < xyz.size(); i++) {
    EXPECT_DOUBLE_EQ(std::get<2>(xyz[i]),
                     estimator.Interpolate(std::make_pair(
                         std::get<0>(xyz[i]), std::get<1>(xyz[i]))));
  }

  EXPECT_DOUBLE_EQ(4.7000000000000002,
                   estimator.Interpolate(std::make_pair(8.5, 14)));
  EXPECT_DOUBLE_EQ(26.292079207920793,
                   estimator.Interpolate(std::make_pair(18.5, 12)));

  // out of range
  EXPECT_DOUBLE_EQ(0.59999999999999998,
                   estimator.Interpolate(std::make_pair(-5, 12)));
  EXPECT_DOUBLE_EQ(30.5, estimator.Interpolate(std::make_pair(30, 12)));
  EXPECT_DOUBLE_EQ(30.5, estimator.Interpolate(std::make_pair(30, -0.5)));
  EXPECT_DOUBLE_EQ(5.4500000000000002,
                   estimator.Interpolate(std::make_pair(10, -0.5)));
  EXPECT_DOUBLE_EQ(5.4500000000000002,
                   estimator.Interpolate(std::make_pair(10, 40)));
  EXPECT_DOUBLE_EQ(30.5, estimator.Interpolate(std::make_pair(40, 40)));
}

TEST_F(Interpolation2DTest, resistance_force_table) {
  const auto& gain_scheduler =
      config_.wheel_torque_optimizer_config().resistance_force_table();
  AINFO << "resistance force table table:" << gain_scheduler.DebugString();

  pnc::Interpolation2D::DataType xyz;

  for (const auto &scheduler : gain_scheduler.scheduler()) {
    xyz.push_back(std::make_tuple(scheduler.speed_kph(),
                                  scheduler.mass_kg(),
                                  scheduler.force_n()));
  }
  pnc::Interpolation2D estimator;
  EXPECT_TRUE(estimator.Init(xyz));

  for (const auto &elem : xyz) {
    EXPECT_DOUBLE_EQ(std::get<2>(elem),
                     estimator.Interpolate(
                         std::make_pair(std::get<0>(elem), std::get<1>(elem))));
  }
}

}  // namespace pnc
}  // namespace xju
