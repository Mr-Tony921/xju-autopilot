/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/util/coordinate.h"

#include "common/math/math_utils.h"

namespace xju {
namespace pnc {

// x_veh, y_veh, theta_veh: vehicle pose in local localization
// x_loc, y_loc, theta_loc: point in local localization
void TransformLocalToVehicleCoord(
    const double x_veh, const double y_veh, const double theta_veh, 
    double* x_loc, double* y_loc, double* theta_loc) {
  double x_offset = *x_loc - x_veh;
  double y_offset = *y_loc - y_veh;
  *x_loc = x_offset * std::cos(theta_veh) + y_offset * std::sin(theta_veh);
  *y_loc = y_offset * std::cos(theta_veh) - x_offset * std::sin(theta_veh);
  if (!theta_loc) {
    return;
  }
  *theta_loc = NormalizeAngle(*theta_loc - theta_veh);
}

// x_veh, y_veh, theta_veh: vehicle pose in local localization
// x_loc, y_loc, theta_loc: point in vehicle coordinate
void TransformVehicleToLocalCoord(
    const double x_veh, const double y_veh, const double theta_veh, 
    double* x_loc, double* y_loc, double* theta_loc) {
  double x_offset = *x_loc * std::cos(theta_veh) - *y_loc * std::sin(theta_veh);
  double y_offset = *y_loc * std::cos(theta_veh) + *x_loc * std::sin(theta_veh);
  *x_loc = x_veh + x_offset;
  *y_loc = y_veh + y_offset;
  if (!theta_loc) {
    return;
  }
  *theta_loc = NormalizeAngle(*theta_loc + theta_veh);
}

// transform "point" from coordinate with point "localize_pose_t1" as the origin
// to coordinate with point "localize_pose_t2" as the origin
PathPoint PathPointPropagate(
    const LocalizePose& localize_pose_t1,
    const LocalizePose& localize_pose_t2,
    const PathPoint& point) {
  PathPoint propagate_point;
  propagate_point.CopyFrom(point);
  double x = point.x();
  double y = point.y();
  double theta = point.theta();
  TransformVehicleToLocalCoord(
      localize_pose_t1.x, localize_pose_t1.y, localize_pose_t1.heading,
      &x, &y, &theta);
  TransformLocalToVehicleCoord(
      localize_pose_t2.x, localize_pose_t2.y, localize_pose_t2.heading,
      &x, &y, &theta);
  propagate_point.set_x(x);
  propagate_point.set_y(y);
  propagate_point.set_theta(theta);
  return propagate_point;
}

// transform "point" from coordinate with point "localize_pose_t1" as the origin
// to coordinate with point "localize_pose_t2" as the origin
void PathPointPropagate(
    const LocalizePose& localize_pose_t1,
    const LocalizePose& localize_pose_t2,
    PathPoint* const point) {
  double x = point->x();
  double y = point->y();
  double theta = point->theta();
  TransformVehicleToLocalCoord(
      localize_pose_t1.x, localize_pose_t1.y, localize_pose_t1.heading,
      &x, &y, &theta);
  TransformLocalToVehicleCoord(
      localize_pose_t2.x, localize_pose_t2.y, localize_pose_t2.heading,
      &x, &y, &theta);
  point->set_x(x);
  point->set_y(y);
  point->set_theta(theta);
}

} // namespace pnc
} // namespace xju