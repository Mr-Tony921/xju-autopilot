/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/gflags/global_gflags.h"

namespace xju {
namespace pnc {

DEFINE_string(vehicle_config_file, "pnc/configs/common/vehicle_config.pb.txt", "vehicle config file.");

DEFINE_string(hdmap_config_file, "pnc/configs/pnc_map/hdmap/Town01.osm", "hdmap config file.");

}  // namespace pnc
}  // namespace xju