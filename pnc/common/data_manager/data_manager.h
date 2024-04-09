/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/design_pattern/singleton.h"
#include "common/data_manager/data_ring_repo.h"
#include "localization.pb.h"

namespace xju {
namespace pnc {

struct LocalizePose {
  double x = 0.0;
  double y = 0.0;
  double heading = 0.0;
  bool is_valid = false;
};

class DataManager {
 public:
  DataRingRepo<LocalizePose> localize_state_repo_ = DataRingRepo<LocalizePose>(50);

 private:
  DataManager() {}
  DECLARE_SINGLETON(DataManager)
};

} // namespace pnc
} // namespace xju
