/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/vehicle_config/vehicle_config_provider.h"

#include "common/gflags/global_gflags.h"
#include "common/file/file.h"
#include "common/logger/logger.h"

namespace xju {
namespace pnc {

bool VehicleConfigProvider::is_init_ = false;
VehicleConfig VehicleConfigProvider::vehicle_config_;

void VehicleConfigProvider::Init() {
  bool status = File::GetProtoConfig<VehicleConfig>(FLAGS_vehicle_config_file, &vehicle_config_);
  // AINFO << vehicle_config_.ShortDebugString();
  if (status) {
    is_init_ = true;
  } else {
    AFATAL << "Vehicle Config File load failed!";
  }
}

const VehicleConfig& VehicleConfigProvider::GetConfig() {
  if (!is_init_) {
    Init();
  }
  return vehicle_config_;
}

} // namespace pnc
} // namespace xju