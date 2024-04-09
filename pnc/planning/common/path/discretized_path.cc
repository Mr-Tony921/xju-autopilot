/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/path/discretized_path.h"

#include <algorithm>

#include "common/logger/logger.h"
#include "common/math/linear_interpolation.h"

namespace xju {
namespace planning {

DiscretizedPath::DiscretizedPath(std::vector<pnc::PathPoint> path_points)
    : std::vector<pnc::PathPoint>(std::move(path_points)) {}

double DiscretizedPath::Length() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

pnc::PathPoint DiscretizedPath::Evaluate(const double s) const {
  ACHECK(!empty());
  auto it_lower = QueryLowerBound(s);
  if (it_lower == begin()) {
    return front();
  }
  if (it_lower == end()) {
    return back();
  }
  return pnc::Interpolate(*(it_lower - 1), *it_lower, s);
}

std::vector<pnc::PathPoint>::const_iterator 
DiscretizedPath::QueryLowerBound(const double s) const {
  auto func = [](const pnc::PathPoint& pt, const double s) {
    return pt.s() < s;
  };
  return std::lower_bound(begin(), end(), s, func);
}

std::vector<pnc::PathPoint>::const_iterator 
DiscretizedPath::QueryUpperBound(const double s) const {
  auto func = [](const double s, const pnc::PathPoint& pt) {
    return pt.s() < s;
  };
  return std::upper_bound(begin(), end(), s, func);
}

} // namespace planning 
} // namespace xju