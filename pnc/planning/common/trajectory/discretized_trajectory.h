/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "common/math/vec2d.h"
#include "pnc_point.pb.h"
#include "planning.pb.h"

namespace xju {
namespace planning {

class DiscretizedTrajectory : public std::vector<pnc::TrajectoryPoint> {
 public:
  DiscretizedTrajectory() = default;
  ~DiscretizedTrajectory() = default;

  explicit DiscretizedTrajectory(const ADCTrajectory& trajectory);
  explicit DiscretizedTrajectory(
      const std::vector<pnc::TrajectoryPoint>& trajectory_points);
  
  void set_trajectory_points(
      const std::vector<pnc::TrajectoryPoint>& trajectory_points);
    
  virtual pnc::TrajectoryPoint StartPoint() const;

  virtual double GetTemporalLength() const;

  virtual double GetSptialLength() const;

  virtual pnc::TrajectoryPoint Evaluate(const double relative_time) const;

  virtual size_t QueryLowerBoundPoint(
      const double relative_time, const double epsilon = 1.0e-5) const;

  virtual size_t QueryNearestPoint(
      const pnc::Vec2d& position) const;

  virtual void AppendTrajectoryPoint(
      const pnc::TrajectoryPoint& trajectory_point);

  virtual void AppendTrajectoryPoints(
      const std::vector<pnc::TrajectoryPoint>& trajectory_points);

  virtual void PrependTrajectoryPoint(
      const pnc::TrajectoryPoint& trajectory_point);

  virtual void PrependTrajectoryPoints(
      const std::vector<pnc::TrajectoryPoint>& trajectory_points);

  const pnc::TrajectoryPoint& TrajectoryPointAt(const size_t index) const;

  size_t NumOfPoints() const { return size(); }

  virtual void Clear() { clear(); }

  void UpdateRelativeTime(const double dt);
};

} // namespace planning
} // namespace xju
