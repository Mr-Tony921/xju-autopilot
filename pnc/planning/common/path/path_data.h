/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "pnc_point.pb.h"
#include "planning/common/path/discretized_path.h"

namespace xju {
namespace planning {

class PathData {
 public:
  PathData() = default;
  ~PathData() = default;

  void set_reference_points(const std::vector<pnc::PathPoint>& points) {
    reference_points_ = points;
  }

  const std::vector<pnc::PathPoint>& reference_points() const {
    return reference_points_;
  }

  void set_planned_path(const std::vector<pnc::PathPoint>& points) {
    planned_path_ = points;
  }

  const std::vector<pnc::FrenetFramePoint>& frenet_points() const {
    return frenet_points_;
  }

  bool set_frenet_points(const std::vector<pnc::FrenetFramePoint>& points);

  const std::vector<pnc::PathPoint>& planned_path() const {
    return planned_path_;
  }

  double TotalLength() const;

  pnc::PathPoint GetPathPointByS(const double s) const;

  std::string DebugString() const;
  
 private:
  bool FrenetPathToCartesianPath();
  void UpdatePlannedPath();
  pnc::PathPoint FrenetPointToCartesianPoint(
      const pnc::FrenetFramePoint& frenet_point,
      const pnc::PathPoint& ref_point);
  
  std::vector<pnc::PathPoint>::const_iterator QueryLowerBound(
    const std::vector<pnc::PathPoint>& points, const double s) const;

  std::vector<pnc::PathPoint>::const_iterator QueryUpperBound(
    const std::vector<pnc::PathPoint>& points, const double s) const;

 private:
  std::vector<pnc::PathPoint> reference_points_;
  std::vector<pnc::PathPoint> planned_path_;
  std::vector<pnc::FrenetFramePoint> frenet_points_;
};

} // namespace planning
} // namespace xju
