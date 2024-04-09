/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>
#include <vector>
#include <limits>
#include <unordered_map>

#include "planning/common/path/path_data.h"
#include "planning/common/obstacle/path_decision.h"
#include "planning/common/speed/speed_data.h"
#include "planning/common/obstacle/obstacle.h"
#include "pnc_map/reference_line.h"
#include "planning/common/path/path_boundary.h"
#include "planning/common/speed/st_graph_data.h"
#include "planning.pb.h"
#include "pnc_point.pb.h"
#include "sl_boundary.pb.h"
#include "vehicle_state.pb.h"
#include "planning/common/trajectory/discretized_trajectory.h"
#include "latency_stats.pb.h"

namespace xju {
namespace planning {

class ReferenceLineInfo {
 public:
  ReferenceLineInfo() = default;
  ~ReferenceLineInfo() = default;

  ReferenceLineInfo(const pnc::VehicleState& vehicle_state,
                    const pnc::TrajectoryPoint& planning_start_point,
                    const pnc_map::ReferenceLine& reference_line);

  bool Init(const std::vector<const Obstacle*>& obstacles);
  bool AddObstacles(const std::vector<const Obstacle*>& obstacles);
  bool IsIrrelevantObstacle(const Obstacle& obstacle);

  const pnc::VehicleState& vehicle_state() const;
  const pnc::TrajectoryPoint& planning_start_point() const;

  const pnc_map::ReferenceLine& reference_line() const;

  const PathData& path_data() const;
  PathData* mutable_path_data();

  const SpeedData& speed_data() const;
  SpeedData* mutable_speed_data();

  const std::vector<PathBoundary>& path_boundary() const;
  std::vector<PathBoundary>* mutable_path_boundary();

  const StGraphData& st_graph_data() const;
  StGraphData* mutable_st_graph_data();
  
  PathDecision* path_decision();
  const PathDecision& path_decision() const;
 
  const double cruise_speed() const;
  void set_cruise_speed(const double speed);
  
  bool is_change_lane() { return is_change_lane_; };
  const bool drivable() const {
    return drivable_;
  }

  void set_drivable(const double drivable) {
    drivable_ = drivable;
  }

  void set_cost(const double cost) {
    cost_ = cost;
  }

  void AddCost(const double cost) {
    cost_ += cost;
  }

  double cost() const {
    return cost_;
  }

  bool const is_adc_on_reference_line() {
    return is_adc_on_reference_line_;
  }

  const std::string path_id() const;
  void set_trajectory(const DiscretizedTrajectory& trajectory);
  const DiscretizedTrajectory& trajectory() const;

  const ADCTrajectory::TrajectoryType& trajectory_type() const {
    return trajectory_type_;
  }
  
  void set_trajectory_type(
      const ADCTrajectory::TrajectoryType trajectory_type) {
    trajectory_type_ = trajectory_type;
  }

  const SLBoundary& car_sl_boundary() const { 
    return car_sl_boundary_; 
  }
  
  bool CombinePathAndSpeedProfile(
      DiscretizedTrajectory* ptr_discretized_trajectory);

  Obstacle* AddObstacle(const Obstacle* obstacle);
  Obstacle* GetBlockingObstacle() const { return blocking_obstacle_; }
  void SetBlockingObstacle(const std::string& blocking_obstacle_id);

  pnc::LatencyStats* mutable_latency_stats() { return &latency_stats_; }
  const pnc::LatencyStats& latency_stats() const { return latency_stats_; }
  
 private:
  bool drivable_ = false;
  double cost_ = 0.0;
  bool is_change_lane_ = false;
  bool is_adc_on_reference_line_ = false;
  double cruise_speed_ = -1.0;
  
  pnc::VehicleState vehicle_state_;
  pnc::TrajectoryPoint planning_start_point_;
  pnc_map::ReferenceLine reference_line_;
  SLBoundary adc_sl_boundary_;
  
  SLBoundary car_sl_boundary_;
  
  PathDecision path_decision_;
  Obstacle* blocking_obstacle_ = nullptr;

  std::vector<PathBoundary> path_boundary_;
  StGraphData st_graph_data_;

  PathData path_data_;
  SpeedData speed_data_;

  DiscretizedTrajectory trajectory_;
  ADCTrajectory::TrajectoryType trajectory_type_;

  pnc::LatencyStats latency_stats_;
};

} // namespace planning
} // namespace xju
