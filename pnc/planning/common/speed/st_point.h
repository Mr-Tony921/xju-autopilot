/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/math/vec2d.h"
namespace xju {
namespace planning {

class STPoint : public pnc::Vec2d {
 public:
  STPoint() = default;
  ~STPoint() = default;
  STPoint(const double s, const double t) 
    : pnc::Vec2d(t, s) {}
  explicit STPoint(const pnc::Vec2d& vec2d_point) 
    : pnc::Vec2d(vec2d_point) {}

  double x() const = delete;
  double y() const = delete;

  double s() const { return y_; }
  double t() const { return x_; }
  void set_s(const double s) { y_ = s; }
  void set_t(const double t) { x_ = t; }
};

} // namespace planning
} // namespace xju
