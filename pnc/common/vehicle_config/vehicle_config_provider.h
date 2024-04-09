/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include "common/design_pattern/singleton.h"
#include "vehicle_config.pb.h"

namespace xju {
namespace pnc {

class VehicleConfigProvider {
 public:
  static const VehicleConfig& GetConfig();

 private:
  static void Init();
 private:
  static bool is_init_;
  static VehicleConfig vehicle_config_;
  DISALLOW_COPY_AND_ASSIGN(VehicleConfigProvider)
};

} // namespace pnc
} // namespace xju
