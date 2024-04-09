/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <list>
#include <queue>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>

#include "pnc_map/lane_process/lane_process.h"
#include "em_lane.pb.h"

namespace xju {
namespace pnc_map {

class ReferenceLineProvider {
 public:
  ReferenceLineProvider() = default;
  ~ReferenceLineProvider() = default;

  // void UpdateVehicleState(const pnc::VehicleState& vehicle_state);

  bool Start() { return false; }

  void Stop() {}

  bool CreateReferenceLines(const std::shared_ptr<pnc::EmLanes>& em_lines,
                            const pnc::LocalizePose& pnc_loc,
                            std::list<ReferenceLine>* reference_lines);

  double time_delay_ms() {
    return time_delay_ms_;
  }

 private:
  pnc::VehicleState vehicle_state_;
  std::list<ReferenceLine> reference_lines_;
  std::queue<std::list<ReferenceLine>> reference_lines_history_;
  double time_delay_ms_;
};

}  // namespace pnc_map
}  // namespace xju
