/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "pnc_point.pb.h"
#include "common/data_manager/data_manager.h"
#include "common/math/math_utils.h"

namespace xju {
namespace pnc {

// x_veh, y_veh, theta_veh: vehicle pose in local localization
// x_loc, y_loc, theta_loc: point in local localization
void TransformLocalToVehicleCoord(
    const double x_veh, const double y_veh, const double theta_veh, 
    double* x_loc, double* y_loc, double* theta_loc=nullptr);

// x_veh, y_veh, theta_veh: vehicle pose in local localization
// x_loc, y_loc, theta_loc: point in vehicle coordinate
void TransformVehicleToLocalCoord(
    const double x_veh, const double y_veh, const double theta_veh, 
    double* x_loc, double* y_loc, double* theta_loc=nullptr);

// transform "point" from coordinate with point "localize_pose_t1" as the origin
// to coordinate with point "localize_pose_t2" as the origin
PathPoint PathPointPropagate(
    const LocalizePose& localize_pose_t1,
    const LocalizePose& localize_pose_t2,
    const PathPoint& point);

// transform "point" from coordinate with point "localize_pose_t1" as the origin
// to coordinate with point "localize_pose_t2" as the origin
void PathPointPropagate(
    const LocalizePose& localize_pose_t1,
    const LocalizePose& localize_pose_t2,
    PathPoint* const point);

} // namespace pnc
} // namespace xju
