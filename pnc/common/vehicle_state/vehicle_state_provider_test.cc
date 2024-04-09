/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/vehicle_state/vehicle_state_provider.h"

#include "gtest/gtest.h"
#include "chassis.pb.h"
#include "planning_config.pb.h"
#include "common/file/file.h"
#include "common/logger/logger.h"
#include "common/math/math_utils.h"

namespace xju {
namespace pnc {

class VehicleStateProviderTest : public ::testing::Test {
 public:
  virtual void SetUp() {
    LoadPlaningConfig();
    LoadInput();
  }

  void LoadPlaningConfig() {
    std::string planning_config_file = "/home/ws/src/pnc/configs/planning/planning.pb.txt";
    std::string gflag_config_file_path = "/home/ws/src/pnc/configs/planning/planning.conf";
    File::GetGflagConfig(gflag_config_file_path);
    File::GetProtoConfig(planning_config_file, &config_);
    VehicleStateProvider::Init(config_.vehicle_state_config());
  }

  void LoadInput() {
    std::string chassis_file = "/home/ws/src/pnc/test_data/chassis.pb.txt";
    std::string chassis_info_file = "/home/ws/src/pnc/test_data/chassis_info.pb.txt";
    std::string localization_file = "/home/ws/src/pnc/test_data/localization.pb.txt";
    File::GetProtoConfig(chassis_file, &chassis_);
    File::GetProtoConfig(chassis_info_file, &chassis_info_);
    File::GetProtoConfig(localization_file, &localization_);
  }

 protected:
  planning::PlanningConfig config_;

  Chassis chassis_;
  ChassisInfo chassis_info_;
  Localization localization_;
};

TEST_F(VehicleStateProviderTest, Update) {
  bool status = VehicleStateProvider::Update(localization_, chassis_info_);
  VehicleState vehicle_state = VehicleStateProvider::GetVehicleState();

  EXPECT_NEAR(VehicleStateProvider::car_x(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::car_y(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::car_heading(), 1.57, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::roll(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::pitch(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::yaw(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::kappa(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::speed(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::acceleration(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::front_wheel_angle(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::mass(), 1400.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::timestamp(), 1173545122.22, kMathEpsilon);
}

TEST_F(VehicleStateProviderTest, Update2) {
  auto linear_velocity = localization_.mutable_linear_velocity();
  linear_velocity->set_y(5.0);
  bool status = VehicleStateProvider::Update(localization_, chassis_info_);
  VehicleState vehicle_state = VehicleStateProvider::GetVehicleState();

  EXPECT_NEAR(VehicleStateProvider::car_x(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::car_y(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::car_heading(), 1.57, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::roll(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::pitch(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::yaw(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::kappa(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::speed(), 5.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::acceleration(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::front_wheel_angle(), 0.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::mass(), 1400.0, kMathEpsilon);
  EXPECT_NEAR(VehicleStateProvider::timestamp(), 1173545122.22, kMathEpsilon);
}

} // namespace pnc
} // namespace xju