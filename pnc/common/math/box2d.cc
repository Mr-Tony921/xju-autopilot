/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/box2d.h"

#include <cmath>
#include <algorithm>

#include "common/math/polygon2d.h"
#include "common/logger/logger.h"
#include "common/math/math_utils.h"

namespace xju {
namespace pnc {

Box2d::Box2d(
    const double center_x, const double center_y,
    const double heading, const double length,
    const double width) 
    : center_(Vec2d(center_x, center_y))
    , center_x_(center_x)
    , center_y_(center_y)
    , heading_(heading)
    , length_(length)
    , width_(width) 
    , half_length_(length / 2.0)
    , half_width_(width / 2.0) {
  CHECK_GT(length_, -kMathEpsilon);
  CHECK_GT(width_, -kMathEpsilon);
  sin_heading_ = std::sin(heading);
  cos_heading_ = std::cos(heading);
  InitCorners();
}

Box2d::Box2d(
    const Vec2d& center,
    const double heading, const double length,
    const double width) 
    : center_(center)
    , center_x_(center.x())
    , center_y_(center.y())
    , heading_(heading)
    , length_(length)
    , width_(width) 
    , half_length_(length / 2.0)
    , half_width_(width / 2.0) {
  CHECK_GT(length_, -kMathEpsilon);
  CHECK_GT(width_, -kMathEpsilon);
  sin_heading_ = std::sin(heading);
  cos_heading_ = std::cos(heading);
  InitCorners();
}

void Box2d::GetAllCorners(std::vector<Vec2d> *const corners) const {
  if (corners == nullptr) {
    return;
  }
  *corners = corners_;
}

bool Box2d::IsOverlap(const Box2d& box) const {
  if (box.max_x() < min_x_ || box.min_x() > max_x_ ||
      box.max_y() < min_y_ || box.min_y() > max_y_) {
      return false;
  }

  const double shift_x = box.center_x() - center_x_;
  const double shift_y = box.center_y() - center_y_;

  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  const double dx3 = box.cos_heading() * box.half_length();
  const double dy3 = box.sin_heading() * box.half_length();
  const double dx4 = box.sin_heading() * box.half_width();
  const double dy4 = -box.cos_heading() * box.half_width();

  return std::abs(shift_x * cos_heading_ + shift_y * sin_heading_) <=
             std::abs(dx3 * cos_heading_ + dy3 * sin_heading_) +
                 std::abs(dx4 * cos_heading_ + dy4 * sin_heading_) +
                 half_length_ &&
         std::abs(shift_x * sin_heading_ - shift_y * cos_heading_) <=
             std::abs(dx3 * sin_heading_ - dy3 * cos_heading_) +
                 std::abs(dx4 * sin_heading_ - dy4 * cos_heading_) +
                 half_width_ &&
         std::abs(shift_x * box.cos_heading() + shift_y * box.sin_heading()) <=
             std::abs(dx1 * box.cos_heading() + dy1 * box.sin_heading()) +
                 std::abs(dx2 * box.cos_heading() + dy2 * box.sin_heading()) +
                 box.half_length() &&
         std::abs(shift_x * box.sin_heading() - shift_y * box.cos_heading()) <=
             std::abs(dx1 * box.sin_heading() - dy1 * box.cos_heading()) +
                 std::abs(dx2 * box.sin_heading() - dy2 * box.cos_heading()) +
                 box.half_width();
}

double Box2d::AccurateDistance(const Box2d& box1, const Box2d& box2) {
  Polygon2d polygon1 = Polygon2d(box1);
  Polygon2d polygon2 = Polygon2d(box2);
  return polygon1.DistanceTo(polygon2);
}

double Box2d::ApproximateDistance(const Box2d& box1, const Box2d& box2) {
  return 0.0;
}

void Box2d::Shift(const double vec_x, const double vec_y) {
  center_x_ += vec_x;
  center_y_ += vec_y;
  InitCorners();
}

void Box2d::RotateFromCenter(const double angle) {
  heading_ = NormalizeAngle(heading_ + angle);
  sin_heading_ = std::sin(heading());
  cos_heading_ = std::cos(heading());
  InitCorners();
}

void Box2d::Extend(const double extension_length, const double extension_width) {
  length_ += extension_length;
  half_length_ += extension_length / 2.0;
  width_ += extension_width;
  half_width_ += extension_width / 2.0;
  InitCorners();
}
  
void Box2d::LongitudinalExtend(const double extension_length) {
  length_ += extension_length;
  half_length_ += extension_length / 2.0;
  InitCorners();
}

void Box2d::LateralExtend(const double extension_width) {
  width_ += extension_width;
  half_width_ += extension_width / 2.0;
  InitCorners();
}

void Box2d::InitCorners() {
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  corners_.clear();
  corners_.emplace_back(center_x_ + dx1 - dx2, center_y_ + dy1 - dy2);
  corners_.emplace_back(center_x_ - dx1 - dx2, center_y_ - dy1 - dy2);
  corners_.emplace_back(center_x_ - dx1 + dx2, center_y_ - dy1 + dy2);
  corners_.emplace_back(center_x_ + dx1 + dx2, center_y_ + dy1 + dy2);

  for (const auto& corner : corners_) {
    min_x_ = std::min(min_x_, corner.x());
    min_y_ = std::min(min_y_, corner.y());
    max_x_ = std::max(max_x_, corner.x());
    max_y_ = std::max(max_y_, corner.y());
  }
}

std::string Box2d::DebugString() const {
  return "box2d ( center = " + center_.DebugString() + "  heading = "  + 
                      std::to_string(heading_) + "  length = " + 
                      std::to_string(length_) + "  width = " + 
                      std::to_string(width_) + " )";
}

} // namespace pnc
} // namespace xju
