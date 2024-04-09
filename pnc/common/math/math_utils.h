/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "pnc_point.pb.h"
#include "common/math/vec2d.h"
#include <vector>

namespace xju {
namespace pnc {

double NormalizeAngle(const double angle);

double AngleDiff(const double from, const double to);

double ToRadian(const double angle);

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1);

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1);

double CrossProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

double InnerProd(const Vec2d &start_point, const Vec2d &end_point_1,
                 const Vec2d &end_point_2);

double CrossProd(const PathPoint &start_point, const PathPoint &end_point_1,
                 const PathPoint &end_point_2);

double InnerProd(const PathPoint &start_point, const PathPoint &end_point_1,
                 const PathPoint &end_point_2);

template <typename T>
inline T Square(const T value) {
  return value * value;
}

double QuaternionToHeading(const double qw, const double qx,
                           const double qy, const double qz);

double Distance(const PathPoint& pt0, const PathPoint& pt1);
double Distance(const TrajectoryPoint& pt0, const TrajectoryPoint& pt1);
double DistanceSquare(const PathPoint& pt0, const PathPoint& pt1);

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template <typename T>
T Clamp(const T value, T bound1, T bound2) {
  if (bound1 > bound2) {
    std::swap(bound1, bound2);
  }

  if (value < bound1) {
    return bound1;
  } else if (value > bound2) {
    return bound2;
  }
  return value;
}

} // namespace pnc
} // namespace xju
