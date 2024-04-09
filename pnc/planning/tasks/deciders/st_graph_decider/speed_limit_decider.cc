/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/tasks/deciders/st_graph_decider/speed_limit_decider.h"

#include <cmath>
#include <limits>

#include "common/math/vec2d.h"
#include "common/vehicle_config/vehicle_config_provider.h"

namespace xju {
namespace planning {

SpeedLimitDecider::SpeedLimitDecider(
    const StGraphDeciderConfig& st_graph_config,
    std::shared_ptr<ReferenceLineInfo> const reference_line_info, Frame* const frame)
    : st_graph_config_(st_graph_config),
      reference_line_info_(reference_line_info),
      frame_(frame) {}

bool SpeedLimitDecider::GetSpeedLimit(SpeedLimit* const speed_limit) {
  const auto& reference_line = reference_line_info_->reference_line();
  const auto& planned_path = 
      reference_line_info_->path_data().planned_path();
  const auto& frenet_points = 
      reference_line_info_->path_data().frenet_points();
  const pnc::VehicleConfig& vehicle_config = pnc::VehicleConfigProvider::GetConfig();
  index_q_.clear();
  if (st_graph_config_.enable_preview_speed_limit()) {
    int num = planned_path.back().s() /
              st_graph_config_.preveiw_speed_limit_delta_s();
    num = std::max(num, 1);
    sliding_window_size_ = planned_path.size() / num;
  }
  for (size_t i = 0; i < frenet_points.size(); ++i) {
    double frenet_point_s = frenet_points.at(i).s();
    // speed limit from map
    double speed_limit_from_reference_line = 
          reference_line.GetSpeedLimitFromS(frenet_point_s);
      // speed limit from centripetal acc
    double speed_limit_from_centripetal_acc = speed_limit_from_reference_line;
    double cal_kappa = planned_path.at(i).kappa();
    if (st_graph_config_.enable_preview_speed_limit()) {
     cal_kappa = MaxKappaInSlidingWindow(planned_path, i);
    }
    if (std::fabs(cal_kappa) > st_graph_config_.minimal_kappa()) {
        speed_limit_from_centripetal_acc =
            std::sqrt(st_graph_config_.max_centric_acceleration_limit() /
                      std::fabs(cal_kappa));
    }
    // ADEBUG <<"std::fabs(planned_path.at(" << i << ").kappa() is: " <<std::fabs(planned_path.at(i).kappa());
    // ADEBUG << "cal_kappa: " << cal_kappa;
    // ADEBUG << "speed_limit_from_centripetal_acc: " << speed_limit_from_centripetal_acc;

    //speed limit from nudge
    double speed_limit_from_nudge = std::numeric_limits<double>::max();
    auto path_decision = reference_line_info_->path_decision();
    for (const auto* obstacle_ptr : path_decision->obstacles().Items()) {
      if(obstacle_ptr->is_virtual()) {
        // ADEBUG << "obstacle: " << obstacle_ptr->id() << " is virtual";
        continue;
      }
      if(!obstacle_ptr->lateral_decision().has_nudge()) {
        continue;
      }
      double veh_rear_axle_to_front = 
          vehicle_config.wheel_base() + 
          vehicle_config.front_overhang_length();

      double veh_front_s = frenet_point_s + veh_rear_axle_to_front;
      double veh_rear_axle_to_back = 
          vehicle_config.back_overhang_length();

      double veh_back_s = frenet_point_s - veh_rear_axle_to_back;
      double obs_front_s = obstacle_ptr->sl_boundary().end_s();
      double obs_back_s = obstacle_ptr->sl_boundary().start_s();
      if (veh_front_s < obs_back_s || veh_back_s > obs_front_s) {
        continue;
      }
      double nudge_speed_ratio = 1.0;
      if (obstacle_ptr->is_static()) {
        nudge_speed_ratio =
            st_graph_config_.static_obs_nudge_speed_ratio();
      } else {
        nudge_speed_ratio = 
            st_graph_config_.dynamic_obs_nudge_speed_ratio();
      }
      speed_limit_from_nudge =
          nudge_speed_ratio * speed_limit_from_reference_line;
      break;
    }      
    double current_speed_limit = 0.0;
    // speed_limit_from_centripetal_acc = speed_limit_from_reference_line;
    if (st_graph_config_.enable_nudge_slow_down()) {
      current_speed_limit = 
          std::fmax(st_graph_config_.lowest_speed(), 
                    std::min({speed_limit_from_nudge,
                             speed_limit_from_centripetal_acc,
                             speed_limit_from_reference_line}));
    } else {
        current_speed_limit = 
          std::fmax(st_graph_config_.lowest_speed(), 
                    std::min({speed_limit_from_centripetal_acc,
                             speed_limit_from_reference_line}));
    }
    speed_limit->AppendSpeedLimit(planned_path.at(i).s(), current_speed_limit);
  }
  return true;
}

double SpeedLimitDecider::MaxKappaInSlidingWindow(
    const std::vector<pnc::PathPoint>& planned_path, const int i) {
  int k = sliding_window_size_;
  if (i > planned_path.size() - k) {
    return planned_path[index_q_.front()].kappa();
  }

  while (!index_q_.empty() && index_q_.front() < i) {
    index_q_.pop_front();
  }
  if (index_q_.empty()) {
    for (int j = i; j < i + k; ++j) {
      if (j > planned_path.size()) {
        break;
      }
      while (!index_q_.empty() &&
               std::fabs(planned_path[j].kappa()) >=
                   std::fabs(planned_path[index_q_.back()].kappa())) {
        index_q_.pop_back();
      }
      index_q_.push_back(j);
    }

  } else if (std::fabs(planned_path[i + k - 1].kappa()) >=
             std::fabs(planned_path[index_q_.front()].kappa())) {
    index_q_.push_front(i + k - 1);
  }
  return planned_path[index_q_.front()].kappa();
}

} // namespace planning
} // namespace xju
