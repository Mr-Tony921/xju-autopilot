/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/trajectory/discretized_trajectory.h"

#include <limits>

#include "common/logger/logger.h"
#include "common/math/linear_interpolation.h"

namespace xju {
namespace planning {

using TrajectoryPoint = pnc::TrajectoryPoint;

DiscretizedTrajectory::DiscretizedTrajectory(
    const ADCTrajectory& trajectory) {
  assign(trajectory.trajectory_point().begin(),
         trajectory.trajectory_point().end());
  if (empty()) {
    AWARN << "trajectory points is empty!";
  }
}
    

DiscretizedTrajectory::DiscretizedTrajectory(
    const std::vector<TrajectoryPoint>& trajectory_points)
    : std::vector<TrajectoryPoint>(trajectory_points) {
  if (trajectory_points.empty()) {
    AWARN << "trajectory points is empty!";
  }
}
  
void DiscretizedTrajectory::set_trajectory_points(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  assign(trajectory_points.begin(), trajectory_points.end());
}
    
TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
  if (empty()) {
    AWARN << "trajectory is empty, return a default construct TrajectoryPoint.";
    TrajectoryPoint tp;
    return tp;
  }
  return front();
}

double DiscretizedTrajectory::GetTemporalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().relative_time() - front().relative_time();
}

double DiscretizedTrajectory::GetSptialLength() const {
   if (empty()) {
    return 0.0;
  }
  return back().path_point().s() - front().path_point().s();
}

TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  auto comp = [](const TrajectoryPoint& pt, const double time) {
    return pt.relative_time() < time;
  };

  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);
  if (it_lower == begin()) {
    return front();
  } else if (it_lower == end()) {
    return back();
  }
  return pnc::Interpolate(*(it_lower - 1), *it_lower, relative_time);
}

size_t DiscretizedTrajectory::QueryLowerBoundPoint(
    const double relative_time, const double epsilon) const {
  ACHECK(!empty());

  if (relative_time >= back().relative_time()) {
    return size() - 1;
  }
  auto comp = [&epsilon](const TrajectoryPoint& tp, 
                        const double time) {
    return tp.relative_time() + epsilon < time;                  
  };
  auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);
  return std::distance(begin(), it_lower);
}

size_t DiscretizedTrajectory::QueryNearestPoint(
    const pnc::Vec2d& position) const {
  double dist_sqrt_min = std::numeric_limits<double>::max();
  size_t index_min = 0;
  for (size_t i = 0; i < size(); ++i) {
    const pnc::Vec2d curr_pt(data()[i].path_point().x(), 
                             data()[i].path_point().y());
    const double dist_sqr = curr_pt.DistanceSquareTo(position);
    if (dist_sqr < dist_sqrt_min) {
      dist_sqrt_min = dist_sqr;
      index_min = i;
    }
  }
  return index_min;
}

void DiscretizedTrajectory::AppendTrajectoryPoint(
    const TrajectoryPoint& trajectory_point) {
  if (!empty()) {
    CHECK_GT(trajectory_point.relative_time(), back().relative_time());
  }
  push_back(trajectory_point);
}

void DiscretizedTrajectory::AppendTrajectoryPoints(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  if (!empty()) {
    CHECK_GT(trajectory_points.front().relative_time(), back().relative_time());
  }
  for (const auto& pt : trajectory_points) {
    push_back(pt);
  }
}

void DiscretizedTrajectory::PrependTrajectoryPoint(
    const pnc::TrajectoryPoint& trajectory_point) {
  if (!empty()) {
    ACHECK(trajectory_point.relative_time() < front().relative_time());
  }
  insert(begin(), trajectory_point);
}

void DiscretizedTrajectory::PrependTrajectoryPoints(
    const std::vector<TrajectoryPoint>& trajectory_points) {
  if (!empty() && trajectory_points.size() > 1) {
    ACHECK(trajectory_points.back().relative_time() <
        front().relative_time());
  }
  insert(begin(), trajectory_points.begin(), trajectory_points.end());
}
      
const TrajectoryPoint& DiscretizedTrajectory::TrajectoryPointAt(
    const size_t index) const {
  CHECK_LT(index, NumOfPoints());
  return data()[index];
}

void DiscretizedTrajectory::UpdateRelativeTime(const double dt) {
  for (size_t i = 0; i < size(); ++i) {
    data()[i].set_relative_time(data()[i].relative_time() + dt);
  }
}

} // namespace planning
} // namespace xju