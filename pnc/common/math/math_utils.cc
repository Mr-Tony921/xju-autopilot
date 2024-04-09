/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/math_utils.h"

#include <cmath>
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include "common/math/euler_angles_zxy.h"
#include "common/math/linear_interpolation.h"

namespace xju {
namespace pnc {

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double AngleDiff(const double from, const double to) {
  return NormalizeAngle(to - from);
}

double ToRadian(const double angle) {
  double a = angle / 180.0 * M_PI;
  return NormalizeAngle(a);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * x1 + y0 * y1;
}

double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

double CrossProd(const PathPoint &start_point, const PathPoint &end_point_1,
                 const PathPoint &end_point_2) {
  double x0 = end_point_1.x() - start_point.x();
  double y0 = end_point_1.y() - start_point.y();
  double x1 = end_point_2.x() - start_point.x();
  double y1 = end_point_2.y() - start_point.y();
  return x0 * y1 - x1 * y0;
}

double InnerProd(const PathPoint &start_point, const PathPoint &end_point_1,
                 const PathPoint &end_point_2) {
  double x0 = end_point_1.x() - start_point.x();
  double y0 = end_point_1.y() - start_point.y();
  double x1 = end_point_2.x() - start_point.x();
  double y1 = end_point_2.y() - start_point.y();
  return x0 * x1 + y0 * y1;
}

double QuaternionToHeading(const double qw, const double qx,
                           const double qy, const double qz) {
  EulerAnglesZXYd euler_angles(qw, qx, qy, qz);
  // euler_angles.yaw() is zero when the car is pointing North, but
  // the heading is zero when the car is pointing East.
  return NormalizeAngle(euler_angles.yaw());
}

double Distance(const PathPoint& pt0, const PathPoint& pt1) {
  return std::hypot(pt0.x() - pt1.x(), pt0.y() - pt1.y());
}

double Distance(const TrajectoryPoint& pt0, const TrajectoryPoint& pt1) {
  return std::hypot(pt0.path_point().x() - pt1.path_point().x(), 
                    pt0.path_point().y() - pt1.path_point().y());
}
} // namespace pnc
} // namespace xju
