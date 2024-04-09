/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/time/time.h"
#include "lane_decider_config.pb.h"
#include "vehicle_config.pb.h"

namespace xju {
namespace planning {

class LaneChangeSpeedDecider {
 public:
  LaneChangeSpeedDecider(const LaneDeciderConfig& lane_decider_config);
  ~LaneChangeSpeedDecider() = default;

  bool Process(const double cruising_speed,
               const std::vector<std::pair<std::array<double, 3>, double>>&
                   target_lane_obs_vec,
               const std::pair<std::array<double, 3>, double>&
                   ego_lane_front_closest_obs,
               const std::array<double, 3>& init_state,
               std::array<double, 3>* const feasible_state);

 private:
  void CalStateSamples(const std::array<double, 3>& init_state,
                       std::vector<std::array<double, 3>>* const state_samples);

  void PredictObsStateConstantV(const std::array<double, 3>& init_state,
                                const double& pred_time,
                                std::array<double, 3>* const final_state);

  void PredictEgoStateConstantAcc(const std::array<double, 3>& init_state,
                                  const std::array<double, 3>& state_sample,
                                  std::array<double, 3>* const final_state);

  bool CheckTargetLaneSpace(
      const std::array<double, 3>& ego_final_state,
      const std::vector<std::pair<std::array<double, 3>, double>>&
          obs_final_state_vec);

  bool CheckEgoLaneFrontSpace(
      const std::array<double, 3>& ego_final_state,
      const std::pair<std::array<double, 3>, double>& front_obs_final_state);

  double CalSampleCost(const std::array<double, 3>& init_state,
                       const std::array<double, 3>& ego_final_state,
                       const double& pred_time);

  LaneDeciderConfig lane_decider_config_;
  pnc::VehicleConfig vehicle_config_;
};

}  // namespace planning
}  // namespace xju
