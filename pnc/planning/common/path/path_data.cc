/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/path/path_data.h"

#include <algorithm>
#include <cmath>
#include <fstream>

#include "common/logger/logger.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"

namespace xju {
namespace planning {

double PathData::TotalLength() const {
  if (planned_path_.empty()) {
    return 0.0;
  }

  return planned_path_.back().s() - planned_path_.front().s();
}

pnc::PathPoint PathData::GetPathPointByS(const double s) const {
  ACHECK(!planned_path_.empty());
  auto it_lower = QueryLowerBound(planned_path_, s);
  if (it_lower == planned_path_.begin()) {
    return planned_path_.front();
  }
  if (it_lower == planned_path_.end()) {
    return planned_path_.back();
  }
  return pnc::Interpolate(*(it_lower - 1), *it_lower, s);
}

bool PathData::set_frenet_points(
    const std::vector<pnc::FrenetFramePoint>& points) {
  frenet_points_ = std::move(points);
  if (reference_points_.empty() || frenet_points_.empty()) {
    AERROR << "Reference points or frenent points is empty.";
    return false;
  }
  if (!FrenetPathToCartesianPath()) {
    AERROR << "Failed to transfer frenet path to cartesian path.";
    return false;
  }
  if (reference_points_.size() != planned_path_.size()) {
    AERROR << "Planned path size is not equal to reference points.";
    return false;
  }
  return true;
}

bool PathData::FrenetPathToCartesianPath() {
  planned_path_.clear();
  if (reference_points_.size() != frenet_points_.size()) {
    AERROR << "Reference path size is not equal to frenent points.";
    return false;
  }
  for (int i = 0; i < reference_points_.size(); ++i) {
    pnc::PathPoint pt =
        FrenetPointToCartesianPoint(frenet_points_[i], reference_points_[i]);
    planned_path_.push_back(pt);
  }
  UpdatePlannedPath();
  return true;
}

pnc::PathPoint PathData::FrenetPointToCartesianPoint(
    const pnc::FrenetFramePoint& frenet_point,
    const pnc::PathPoint& ref_point) {
  pnc::PathPoint pt;

  pt.set_x(ref_point.x() - frenet_point.l() * std::sin(ref_point.theta()));
  pt.set_y(ref_point.y() + frenet_point.l() * std::cos(ref_point.theta()));
  pt.set_s(ref_point.s());
  pt.set_theta(pnc::NormalizeAngle(ref_point.theta() +
                                   frenet_point.heading_error()));
  pt.set_kappa(frenet_point.kappa());
  if (frenet_point.has_dkappa()) {
    pt.set_dkappa(frenet_point.dkappa());
  }
  return pt;
}

void PathData::UpdatePlannedPath() {
  pnc::PathPoint& pt = planned_path_[0];
  if (!pt.has_dkappa()) {
    if (reference_points_[0].has_dkappa()) {
      pt.set_dkappa(reference_points_[0].dkappa());
    } else {
      pt.set_dkappa(0.0);
    }
  }
  pt.set_ddkappa(0.0);

  double s = 0.0;
  planned_path_[0].set_s(s);
  for (int i = 1; i < planned_path_.size(); ++i) {
    double delta_s = pnc::Distance(planned_path_[i - 1], planned_path_[i]);
    s += delta_s;
    planned_path_[i].set_s(s);
    if (!planned_path_[i].has_dkappa()) {
      double dkappa =
          (planned_path_[i].kappa() - planned_path_[i - 1].kappa()) / delta_s;
      planned_path_[i].set_dkappa(dkappa);
    }

    double ddkappa =
        (planned_path_[i].dkappa() - planned_path_[i - 1].dkappa()) / delta_s;
    planned_path_[i].set_ddkappa(ddkappa);
  }
}

std::vector<pnc::PathPoint>::const_iterator PathData::QueryLowerBound(
    const std::vector<pnc::PathPoint>& points, const double s) const {
  auto func = [](const pnc::PathPoint& pt, const double s) {
    return pt.s() < s;
  };
  return std::lower_bound(points.begin(), points.end(), s, func);
}

std::vector<pnc::PathPoint>::const_iterator PathData::QueryUpperBound(
    const std::vector<pnc::PathPoint>& points, const double s) const {
  auto func = [](const double s, const pnc::PathPoint& pt) {
    return pt.s() < s;
  };
  return std::upper_bound(points.begin(), points.end(), s, func);
}

std::string PathData::DebugString() const {
  std::stringstream ss;
  ss << "planed_path size: " << planned_path_.size() << "\n";
  for (int i = 0; i < planned_path_.size(); i++) {
    ss << "[" << i << "]  x: " << planned_path_[i].x()
       << "  y: " << planned_path_[i].y() << "  z: " << planned_path_[i].z()
       << "  theta: " << planned_path_[i].theta()
       << "  k: " << planned_path_[i].kappa()
       << "  dk: " << planned_path_[i].dkappa()
       << "  ddk: " << planned_path_[i].ddkappa()
       << "  s: " << planned_path_[i].s() << "\n";
  }
  return ss.str();
}

}  // namespace planning
}  // namespace xju
