/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>

#include "common/design_pattern/singleton.h"
#include "common/filter/mean_filter.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "localization.pb.h"
#include "chassis.pb.h"
#include "vehicle_state.pb.h"
#include "vehicle_signal.pb.h"
#include "common/data_manager/data_manager.h"

namespace xju {
namespace pnc {

class VehicleStateProvider {
 public:
  static void Init(const VehicleStateConfig& config);
  static void Clear();

  static bool Update(
      const Localization& localization,
      const ChassisInfo& chassis_info);

  static const VehicleState& GetVehicleState();

  static const LocalizePose localize_pose();
  
  static const double car_x();
  static const double car_y();
  static const double car_heading();

  static const double roll();
  static const double pitch();
  static const double yaw();
  static const double kappa();
  static const double speed();
  static const double acceleration();
  static const double front_wheel_angle();
  static const double steering_angle();
  static const double mass();
  static const double timestamp();

 private:
  static bool ConstructFromLocalization(
      const Localization& localization, const ChassisInfo& chassis_info);
  static bool ConstructFromChassis(const ChassisInfo& chassis_info);
  static bool IsCarGoStraight();

 private:
  static VehicleState vehicle_state_;
  static VehicleState last_vehicle_state_;
  // config
  static VehicleConfig vehicle_config_;
  // kalmen filter config
  static VehicleStateConfig vehicle_state_config_;
  // Covariance of current state belief dist
  static double p_;
  // mean filter
  static MeanFilter mean_filter_;
  // go straight count
  static double go_straight_distance_;
  static bool is_first_compute_;

  // localize
  static LocalizePose localize_pose_;
  static LocalizePose last_localize_pose_;

  DISALLOW_COPY_AND_ASSIGN(VehicleStateProvider)
};

} // namespace pnc
} // namespace xju
