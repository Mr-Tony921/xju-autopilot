/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <utility>
#include <vector>

#include "pnc_point.pb.h"

namespace xju {
namespace pnc {

class PathMatcher {
 public:
  PathMatcher() = delete;

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double x, const double y);

  static std::pair<double, double> GetPathFrenetCoordinate(
      const std::vector<PathPoint>& reference_line, const double x,
      const double y);

  static PathPoint MatchToPath(const std::vector<PathPoint>& reference_line,
                               const double s);

 private:
  static PathPoint FindProjectionPoint(const PathPoint& p0, const PathPoint& p1,
                                       const double x, const double y);
};

} // namespace pnc
} // namespace xju
