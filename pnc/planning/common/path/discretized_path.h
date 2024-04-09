/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "pnc_point.pb.h"

namespace xju {
namespace planning {

class DiscretizedPath : public std::vector<pnc::PathPoint> {
 public:
  DiscretizedPath() = default;
  ~DiscretizedPath() = default;

  explicit DiscretizedPath(std::vector<pnc::PathPoint> path_points);

  double Length() const;

  pnc::PathPoint Evaluate(const double s) const;

 protected:
  std::vector<pnc::PathPoint>::const_iterator QueryLowerBound(const double s) const;
  std::vector<pnc::PathPoint>::const_iterator QueryUpperBound(const double s) const;
};

} // namespace planning 
} // namespace xju
