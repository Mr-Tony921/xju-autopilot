/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cmath>

#include "common/logger/logger.h"
#include "common/math/math_utils.h"
#include "pnc_point.pb.h"

namespace xju {
namespace pnc {
template <typename T>
T lerp(const T& x0, const double t0, const T& x1, const double t1,
       const double t) {
  if (std::fabs(t1 - t0) <= kMathEpsilon) {
    AERROR << "input time difference is too small";
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const T x = x0 + r * (x1 - x0);
  return x; 
}

template <typename T>
T lerp(const T& x0, const T& x1, const double t) {
  if (std::fabs(t) <= kMathEpsilon) {
    AERROR << "input time difference is too small";
    return x0;
  }
  const T x = x0 + t * (x1 - x0);
  return x; 
}

double AngleSlerp(const double a0, const double t0, const double a1, 
                  const double t1, const double t);

double AngleSlerp(const double a0, const double a1, 
                  const double t);

SLPoint Interpolate(const SLPoint& p0, 
                    const SLPoint& p1, const double w);

PathPoint InterpolateByRatio(
    const PathPoint& p0, const PathPoint& p1, const double r);

PathPoint Interpolate(const PathPoint& p0, 
                      const PathPoint& p1, const double s);

TrajectoryPoint Interpolate(const TrajectoryPoint& p0, 
                            const TrajectoryPoint& p1, const double t);

} // namespace pnc
} // namespace xju
