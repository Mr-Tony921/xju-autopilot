/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/math/box2d.h"
#include "planning/common/frame/frame.h"
#include "planning/common/obstacle/obstacle.h"
#include "planning/common/path/path_data.h"
#include "planning/common/reference_line_info/reference_line_info.h"
#include "planning/common/speed/st_boundary.h"
#include "vehicle_config.pb.h"

namespace xju {
namespace planning {
class StBoundaryMapper {
 public:
  StBoundaryMapper(
      const StGraphDeciderConfig& st_graph_decider_config,
      std::shared_ptr<ReferenceLineInfo> const reference_line_info, 
      Frame* const frame); 

  ~StBoundaryMapper() = default;

  bool MapObstaclesToSTBoundaries();

 private:
  void ComputeSTBoundary(Obstacle* const obstacle_ptr);
  
  void ComputeSTBoundaryWithDecision(
    const ObjectDecisionType& decision, 
    Obstacle* const obstacle) const;
  void GenerateStopStBoundary(double lower_s, Obstacle* const obstacle_ptr);

  bool GetOverlapBoundaryPoints(
    const std::vector<pnc::PathPoint>& planned_path, 
    const Obstacle& obstacle, 
    std::vector<STPoint>* const upper_st_points, 
    std::vector<STPoint>* const lower_st_points);

  double CalculateBoxDistance(const pnc::PathPoint& path_point, 
                              const pnc::Box2d& obs_box);

  bool CheckOverlap(const pnc::PathPoint& path_point, 
                    const pnc::Box2d&  obs_box);

  pnc::Box2d GenerateCarBox(
    const pnc::PathPoint& path_point, const double l_buffer);

 private:
  StBoundaryMapper() = default;
  StBoundaryMapper(const StBoundaryMapper& st_boundary_mapper);
  const StGraphDeciderConfig& st_graph_config_;
  std::shared_ptr<ReferenceLineInfo> reference_line_info_;
  Frame* frame_;
  PathData path_data_;
  pnc::VehicleConfig vehicle_config_;

};
} // namespace planning
    
} // namespace xju
