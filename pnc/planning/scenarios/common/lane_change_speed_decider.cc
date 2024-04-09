/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/scenarios/common/lane_change_speed_decider.h"

#include "common/file/file.h"
#include "common/logger/logger.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace xju {
namespace planning {

using State = std::array<double, 3>; // (s, v, a)
using ObsState = std::pair<State, double>;  // ((s, v, a）, length) 
using StateSample = std::array<double, 3>; // (a, v, time)

LaneChangeSpeedDecider::LaneChangeSpeedDecider(
    const LaneDeciderConfig& lane_decider_config)
    : lane_decider_config_(lane_decider_config) {
    vehicle_config_ = pnc::VehicleConfigProvider::GetConfig();
}

bool LaneChangeSpeedDecider::Process(
    const double cruising_speed,
    const std::vector<ObsState>& target_lane_obs_vec,
    const ObsState& ego_lane_front_closest_obs,
    const State& init_state, StateSample* const feasible_state) {
  if (target_lane_obs_vec.empty()) {
    return true;
  }
  
  double start = pnc::Time::NowInSeconds();
  bool ret = false;
  
  /* Calculate state samples */
  std::vector<StateSample> state_samples;
  CalStateSamples(init_state, &state_samples);
  ADEBUG << "state_samples size : " << state_samples.size();
  // for (auto& sample : state_samples) {
  //   ADEBUG << "state_samples, [a, v, t]: [" << sample[0]
  //          << ", " << sample[1] << ", " << sample[2] << "]";
  // }
  
  // std::vector<std::pair<StateSample, double>> feasible_samples;
  double min_cost = std::numeric_limits<float>::max();

  for (const auto& sample_i : state_samples) {
    /* Calculate final state of obstacles in target lane */
    std::vector<ObsState> target_obs_final_state_vec;
    for (const auto& obs_i : target_lane_obs_vec) {
      State obs_final_state;
      PredictObsStateConstantV(obs_i.first, sample_i[2], &obs_final_state);
      target_obs_final_state_vec.emplace_back(obs_final_state, obs_i.second);
    }

    /* Calculate final state of front closest obstacle in ego lane */
    ObsState front_obs_final_state;
    if (ego_lane_front_closest_obs.first[0] <= init_state[0] ||
        ego_lane_front_closest_obs.second < 1e-6) {
      front_obs_final_state.first = {std::numeric_limits<float>::max(),
                                     std::numeric_limits<float>::max(), 0.0};
      front_obs_final_state.second = ego_lane_front_closest_obs.second;
    } else {
      PredictObsStateConstantV(ego_lane_front_closest_obs.first, sample_i[2],
                               &front_obs_final_state.first);
      front_obs_final_state.second = ego_lane_front_closest_obs.second;
    }

    /* Calculate final state of ego vehicle */
    State ego_final_state;
    PredictEgoStateConstantAcc(init_state, sample_i, &ego_final_state);

    if (CheckTargetLaneSpace(ego_final_state, target_obs_final_state_vec) &&
         CheckEgoLaneFrontSpace(ego_final_state, front_obs_final_state)) {
      /* Calulate state cost */
      double sample_cost =
          CalSampleCost(init_state, ego_final_state, sample_i[2]);
      // feasible_samples.emplace_back(sample_i, sample_cost);
      if (sample_cost < min_cost) {
        min_cost = sample_cost;
        *feasible_state = sample_i;
        ret = true;
      }
    }
  }
  // ADEBUG << "feasible_samples.size(): " << feasible_samples.size();
  // for (auto& sample : feasible_samples) {
  //   ADEBUG << "feasible_sample [a, v, t]: [" << sample.first[0]
  //          << ", " << sample.first[1] << ", " << sample.first[2]
  //          << "], cost: " << sample.second;
  // }
  if (!ret) {
    ADEBUG << "LaneChangeSpeedDecider: Can't find feasible space!";
  }
  ADEBUG << "feasible_state [a, v, t]: [" << (*feasible_state)[0]
         << ", " << (*feasible_state)[1] << ", " << (*feasible_state)[2]
         << "], cost: " << min_cost;
  double cost_time = (pnc::Time::NowInSeconds() - start) * 1000.0;
  ADEBUG << "Cost time of LaneChangeSpeedDecider is: " << cost_time << "ms.";

  return ret;
}

void LaneChangeSpeedDecider::CalStateSamples(
    const State& init_state, 
    std::vector<StateSample>* const state_samples) {
    // 1. target acceleration sampling
  double acc_sample_range = lane_decider_config_.prepare_accel_upper() -
                            lane_decider_config_.prepare_accel_lower();
  double acc_sample_step = lane_decider_config_.prepare_accel_step();
  auto acc_sample_length =
      static_cast<unsigned int>(acc_sample_range / acc_sample_step);
  std::vector<double> acc_sample_states;
  for (unsigned int i = 0; i <= acc_sample_length; i++) {
    double acc_sample_state =
        lane_decider_config_.prepare_accel_lower() + i * acc_sample_step;
    if (acc_sample_state <= lane_decider_config_.prepare_accel_upper() &&
        std::abs(acc_sample_state) > 1e-6) {
      acc_sample_states.emplace_back(acc_sample_state);
      // ADEBUG << "acc_sample: " << acc_sample_state;
    }
  }
  // 2. target speed sampling
  double v_sample_lower =
      init_state[1] * lane_decider_config_.prepare_speed_lower_factor();
  double v_sample_upper = std::min(
      init_state[1] * lane_decider_config_.prepare_speed_upper_factor(),
      lane_decider_config_.ego_speed_limit_lane_change());
  double v_sample_range = v_sample_upper - v_sample_lower;
  double v_sample_step = lane_decider_config_.prepare_speed_step();
  auto v_sample_length =
      static_cast<unsigned int>(v_sample_range / v_sample_step);
  std::vector<double> v_sample_states;
  for (unsigned int i = 0; i <= v_sample_length; i++) {
    double v_sample_state = v_sample_lower + i * v_sample_step;
    if (v_sample_state <= v_sample_upper) {
      v_sample_states.emplace_back(v_sample_state);
      // ADEBUG << "v_sample: " << v_sample_state;
    }
  }
  // 3. adjust time sampling
  double t_sample_range = lane_decider_config_.adjust_time_upper() -
                          lane_decider_config_.adjust_time_lower();
  double t_sample_step = lane_decider_config_.adjust_time_step();
  auto t_sample_length =
      static_cast<unsigned int>(t_sample_range / t_sample_step);
  std::vector<double> t_sample_states;
  for (unsigned int i = 0; i <= t_sample_length; i++) {
    double t_sample_state =
        lane_decider_config_.adjust_time_lower() + i * t_sample_step;
    if (t_sample_state <= lane_decider_config_.adjust_time_upper()) {
      t_sample_states.emplace_back(t_sample_state);
      // ADEBUG << "t_sample: " << t_sample_state;
    }
  }
  // 4. obtain target sampling combinations that meet the conditions
  for (auto acc_target : acc_sample_states) {
    for (auto v_target : v_sample_states) {
      if ((acc_target > 0 && v_target <= init_state[1]) ||
          (acc_target < 0 && v_target >= init_state[1])) {
        continue;
      }
      for (auto adjust_time : t_sample_states) {
        if (std::abs((v_target - init_state[1]) / acc_target) > adjust_time) {
          continue;
        }
        StateSample state_target;
        state_target[0] = acc_target;
        state_target[1] = v_target;
        state_target[2] = adjust_time;
        state_samples->emplace_back(state_target);
      }
    }
  }
  for (auto adjust_time : t_sample_states) {
    state_samples->emplace_back(StateSample({0.0, init_state[1], adjust_time}));
  }
}

void LaneChangeSpeedDecider::PredictObsStateConstantV(
    const State& init_state, const double& pred_time,
    State* const final_state) {
  // final state : s = s0 + v0 * t
  (*final_state)[0] = init_state[0] + init_state[1] * pred_time;
  // final state : v = v0
  (*final_state)[1] = init_state[1];
  // final state : a = 0
  (*final_state)[2] = 0.0;
}

void LaneChangeSpeedDecider::PredictEgoStateConstantAcc(
    const State& init_state, const StateSample& state_sample,
    State* const final_state) {
  double a_target = state_sample[0];
  double v_target = state_sample[1];
  double adjust_time = state_sample[2];
  double s_init = init_state[0];
  double v_init = init_state[1];
  // calculate actual time spent achieving target speed
  if (a_target == 0.0) {
    (*final_state)[0] = s_init + v_init * adjust_time;
    (*final_state)[1] = v_init;
    (*final_state)[2] = 0.0;
  } else {
    double actual_time = (v_target - v_init) / a_target;
    if (actual_time >= adjust_time) {
      // final state : s = s0 + v0 * actual_time +
      //                   0.5 * a_target * actual_time^2
      (*final_state)[0] = s_init + v_init * adjust_time +
                          0.5 * a_target * adjust_time * adjust_time;
      // final state : v = v0 + a_target * t
      (*final_state)[1] = v_init + a_target * adjust_time;
      // final state : a = a_target
      (*final_state)[2] = a_target;
    } else {
      // final state : s = s0 + v0 * actual_time +
      //                   0.5 * a_target * actual_time^2 +
      //                   v_target * (adjust_time - actual_time)
      (*final_state)[0] = s_init + v_init * actual_time +
                          0.5 * a_target * actual_time * actual_time +
                          v_target * (adjust_time - actual_time);
      // final state : v = v_target
      (*final_state)[1] = v_target;
      // final state : a = a_target
      (*final_state)[2] = a_target;
    }
  }
}

bool LaneChangeSpeedDecider::CheckTargetLaneSpace(
    const State& ego_final_state,
    const std::vector<ObsState>& obs_final_state_vec) {
  // calculate the upper and lower limits of the safety distance
  double dist_buffer = lane_decider_config_.space_hysteresis_buffer();
  double max_safe_dist_back =
      ego_final_state[1] * lane_decider_config_.max_safe_distance_back_time() +
      lane_decider_config_.min_obst_dist_back() + dist_buffer;
  double min_safe_dist_back =
      ego_final_state[1] * lane_decider_config_.min_safe_distance_back_time() +
      lane_decider_config_.min_obst_dist_back() + dist_buffer;
  double max_safe_dist_front =
      ego_final_state[1] * lane_decider_config_.max_safe_distance_front_time() +
      lane_decider_config_.min_obst_dist_front() + dist_buffer;
  double min_safe_dist_front =
      ego_final_state[1] * lane_decider_config_.min_safe_distance_front_time() +
      lane_decider_config_.min_obst_dist_front() + dist_buffer;
  double ego_front_edge_s = ego_final_state[0] +
                            vehicle_config_.front_edge_to_center() +
                            vehicle_config_.rear_axle_to_center();
  double ego_back_edge_s = 0.0;
  ego_back_edge_s =
      ego_final_state[0] - (vehicle_config_.back_edge_to_center() -
                            vehicle_config_.rear_axle_to_center());

  // traverse target lane obstacles state
  for (auto obs_state : obs_final_state_vec) {
    double obs_front_edge_s = obs_state.first[0] + 0.5 * obs_state.second;
    double obs_back_edge_s = obs_state.first[0] - 0.5 * obs_state.second;
    // target lane obstacle is behind of ego or overlap
    // with ego in longitudinal position
    if (obs_state.first[0] < ego_final_state[0]) {
      double delta_s = ego_back_edge_s - obs_front_edge_s;
      if (delta_s < min_safe_dist_back) {
        return false;
      } else if (delta_s <= max_safe_dist_back) {
        double min_safe_speed_ratio =
            1.0 - lane_decider_config_.near_safe_speed_diff_ratio();
        double max_safe_speed_ratio =
            1.0 + lane_decider_config_.far_safe_speed_diff_ratio();
        double safefy_speed_max_back_obs =
            (ego_final_state[1] * min_safe_speed_ratio -
             ego_final_state[1] * max_safe_speed_ratio) /
                (max_safe_dist_back - min_safe_dist_back) *
                (max_safe_dist_back - delta_s) +
            ego_final_state[1] * max_safe_speed_ratio;
        if (obs_state.first[1] > safefy_speed_max_back_obs) {
          // obs is behind of ego and speed is too fast
          return false;
        }
      }
    }
    // target lane obstacle is front of ego or overlap
    // with ego in longitudinal position
    if (obs_state.first[0] > ego_final_state[0]) {
      double delta_s = obs_back_edge_s - ego_front_edge_s;
      if (delta_s < min_safe_dist_front) {
        return false;
      } else if (delta_s <= max_safe_dist_front) {
        double min_safe_speed_ratio =
            1.0 - lane_decider_config_.far_safe_speed_diff_ratio();
        double max_safe_speed_ratio =
            1.0 + lane_decider_config_.near_safe_speed_diff_ratio();
        double safefy_speed_min_front_obs =
            (ego_final_state[1] * max_safe_speed_ratio -
             ego_final_state[1] * min_safe_speed_ratio) /
                (max_safe_dist_front - min_safe_dist_front) *
                (max_safe_dist_front - delta_s) +
            ego_final_state[1] * min_safe_speed_ratio;
        if (obs_state.first[1] < safefy_speed_min_front_obs) {
          // obs in front of ego and speed is too slow
          return false;
        }
      }
    }
  }
  return true;
}

bool LaneChangeSpeedDecider::CheckEgoLaneFrontSpace(
    const State& ego_final_state, const ObsState& front_obs_final_state) {
  double real_dist_with_front_obs =
      front_obs_final_state.first[0] - front_obs_final_state.second * 0.5 -
      (ego_final_state[0] +
       vehicle_config_.front_edge_to_center() +
       vehicle_config_.rear_axle_to_center());
  double t1 = lane_decider_config_.front_obst_delta_v_coef();
  double t2 = lane_decider_config_.front_obst_v_coef();
  double safe_dist =
      (ego_final_state[1] - front_obs_final_state.first[1]) * t1 +
      ego_final_state[1] * t2 + lane_decider_config_.min_obst_dist_front();
  double max_safe_dist_with_front_obs =
      std::max(safe_dist, lane_decider_config_.min_obst_dist_front());
  if (real_dist_with_front_obs < max_safe_dist_with_front_obs) return false;
  return true;
}

double LaneChangeSpeedDecider::CalSampleCost(const State& init_state,
                                             const State& ego_final_state,
                                             const double& pred_time) {
  double omega_acc = lane_decider_config_.weight_accel();
  double omega_v = lane_decider_config_.weight_speed();
  double omega_t = lane_decider_config_.weight_time();

  double total_cost = omega_acc * std::abs(ego_final_state[2]) +
                      omega_v * std::abs(ego_final_state[1] - init_state[1]) +
                      omega_t * pred_time;
  return total_cost;
}

}  // namespace planning
}  // namespace xju

