/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/path_matcher.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "common/logger/logger.h"
#include "common/math/linear_interpolation.h"

namespace xju {
namespace pnc {

PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double x, const double y) {
  CHECK_GT(reference_line.size(), 0);

  auto func_distance_square = [](const PathPoint& point, const double x,
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double distance_min = func_distance_square(reference_line.front(), x, y);
  std::size_t index_min = 0;

  for (std::size_t i = 1; i < reference_line.size(); ++i) {
    double distance_temp = func_distance_square(reference_line[i], x, y);
    if (distance_temp < distance_min) {
      distance_min = distance_temp;
      index_min = i;
    }
  }

  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;
  std::size_t index_end =
      (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;

  if (index_start == index_end) {
    return reference_line[index_start];
  }

  return FindProjectionPoint(reference_line[index_start],
                             reference_line[index_end], x, y);
}

std::pair<double, double> PathMatcher::GetPathFrenetCoordinate(
    const std::vector<PathPoint>& reference_line, const double x,
    const double y) {
  auto matched_path_point = MatchToPath(reference_line, x, y);
  double rtheta = matched_path_point.theta();
  double rx = matched_path_point.x();
  double ry = matched_path_point.y();
  double delta_x = x - rx;
  double delta_y = y - ry;
  double side = std::cos(rtheta) * delta_y - std::sin(rtheta) * delta_x;
  std::pair<double, double> relative_coordinate;
  relative_coordinate.first = matched_path_point.s();
  relative_coordinate.second =
      std::copysign(std::hypot(delta_x, delta_y), side);
  return relative_coordinate;
}

PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double s) {
  auto comp = [](const PathPoint& point, const double s) {
    return point.s() < s;
  };

  auto it_lower =
      std::lower_bound(reference_line.begin(), reference_line.end(), s, comp);
  if (it_lower == reference_line.begin()) {
    return reference_line.front();
  } else if (it_lower == reference_line.end()) {
    return reference_line.back();
  }

  return Interpolate(*(it_lower - 1), *it_lower, s);
}

PathPoint PathMatcher::FindProjectionPoint(const PathPoint& p0,
                                           const PathPoint& p1, const double x,
                                           const double y) {
  double v0x = x - p0.x();
  double v0y = y - p0.y();

  double v1x = p1.x() - p0.x();
  double v1y = p1.y() - p0.y();

  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
  double dot = v0x * v1x + v0y * v1y;

  double delta_s = dot / v1_norm;
  return Interpolate(p0, p1, p0.s() + delta_s);
}

} // namespace pnc
} // namespace xju