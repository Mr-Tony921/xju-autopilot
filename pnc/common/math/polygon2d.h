/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <limits>

#include "common/math/box2d.h"

namespace xju {
namespace pnc {

class Polygon2d {
 public:
  Polygon2d() = default;
  ~Polygon2d() = default;
  
  explicit Polygon2d(const std::vector<Vec2d>& points);
  explicit Polygon2d(const Box2d& box);

  bool IsOverlap(Polygon2d& polygon);

  bool IsOverlap(const Box2d& box);

  bool is_convex() const {
    return is_convex_;
  }

  bool ComputeConvexHull();

  double DistanceTo(const Polygon2d& polygon);

  int num_of_corners() const {
    return num_of_corners_;
  }

  const std::vector<Vec2d>& corners() const {
    return corners_;
  }

  double min_x() const {
    return min_x_;
  }

  double max_x() const {
    return max_x_;
  }

  double min_y() const {
    return min_y_;
  }

  double max_y() const {
    return max_y_;
  }

 protected:
  void BuildFromPoints();
  bool GjkLevel1(const Polygon2d& polygon);
  bool GjkLevel1(const Box2d& box);
  double GjkLevel2(const Polygon2d& polygon);
  int Next(int at) const;
  int Prev(int at) const;

  bool is_convex_ = false;
  int num_of_corners_;
  std::vector<Vec2d> corners_;
  
  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();
};

} // namespace pnc
} // namespace xju
