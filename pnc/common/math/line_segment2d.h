/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/math/vec2d.h"

namespace xju {
namespace pnc {

class LineSegment2d {
 public:
  LineSegment2d();

  LineSegment2d(const Vec2d &start, const Vec2d &end);

  const Vec2d &start() const { return start_; }

  const Vec2d &end() const { return end_; }

  const Vec2d &unit_direction() const { return unit_direction_; }

  Vec2d center() const { return (start_ + end_) / 2.0; }

  Vec2d rotate(const double angle);

  double heading() const { return heading_; }

  double cos_heading() const { return unit_direction_.x(); }

  double sin_heading() const { return unit_direction_.y(); }

  double length() const;

  double length_sqr() const;

  double DistanceTo(const Vec2d &point) const;

  double DistanceTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  double DistanceSquareTo(const Vec2d &point) const;

  double DistanceSquareTo(const Vec2d &point, Vec2d *const nearest_pt) const;

  bool IsPointIn(const Vec2d &point) const;

  bool HasIntersect(const LineSegment2d &other_segment) const;

  bool GetIntersect(const LineSegment2d &other_segment,
                    Vec2d *const point) const;

  double ProjectOntoUnit(const Vec2d &point) const;

  double ProductOntoUnit(const Vec2d &point) const;

  double GetPerpendicularFoot(const Vec2d &point,
                              Vec2d *const foot_point) const;


 private:
  Vec2d start_;
  Vec2d end_;
  Vec2d unit_direction_;
  double heading_ = 0.0;
  double length_ = 0.0;
};  
} // namespace pnc
} // namespace xju
