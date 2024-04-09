/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <limits>

#include "common/math/vec2d.h"

namespace xju {
namespace pnc {

class Box2d {
 public:
  Box2d() = default;
  ~Box2d() = default;

  Box2d(const double center_x, const double center_y,
        const double heading, const double length,
        const double width);

  Box2d(const Vec2d& center,
        const double heading, const double length,
        const double width);
  
  bool IsOverlap(const Box2d& box) const;

  static double AccurateDistance(const Box2d& box1, const Box2d& box2);
  static double ApproximateDistance(const Box2d& box1, const Box2d& box2);
  
  // static double Distance(const Box2d& box1, const Box2d& box2);
  void Shift(const double vec_x, const double vec_y);

  void RotateFromCenter(const double angle);

  void Extend(const double extension_length, const double extension_width);
  
  void LongitudinalExtend(const double extension_length);

  void LateralExtend(const double extension_width);

  const std::vector<Vec2d>& corners() const {
    return corners_;
  }

  void GetAllCorners(std::vector<Vec2d> *const corners) const;

  const Vec2d center() const {
    return center_;
  }

  const double center_x() const {
    return center_x_;
  }

  const double center_y() const {
    return center_y_;
  }

  const double length() const {
    return length_;
  }

  const double width() const {
    return width_;
  }

  const double heading() const {
    return heading_;
  }

  const double half_length() const {
    return half_length_;
  }

  const double half_width() const {
    return half_width_;
  }

  const double sin_heading() const {
    return sin_heading_;
  }

  const double cos_heading() const {
    return cos_heading_;
  }

  const double min_x() const {
    return min_x_;
  }

  const double min_y() const {
    return min_y_;
  }

  const double max_x() const {
    return max_x_;
  }

  const double max_y() const {
    return max_y_;
  }

  std::string DebugString() const;

 private:
  void InitCorners();

 private:
  Vec2d center_;
  double center_x_ = .0;
  double center_y_ = .0;
  double heading_ = .0;
  double length_ = .0;
  double width_ = .0;
  double half_length_ = .0;
  double half_width_ = .0;
  double cos_heading_ = .0;
  double sin_heading_ = .0;

  std::vector<Vec2d> corners_;

  double max_x_ = std::numeric_limits<double>::lowest();
  double min_x_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::lowest();
  double min_y_ = std::numeric_limits<double>::max();

};

} // namespace pnc
} // namespace xju

