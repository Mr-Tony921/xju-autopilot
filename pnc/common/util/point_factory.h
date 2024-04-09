/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/math/vec2d.h"
#include "pnc_point.pb.h"

namespace xju {
namespace pnc {
class PointFactory {
 public:
  template <typename XY>
  static Vec2d ToVec2d(const XY& xy) {
    return Vec2d(xy.x(), xy.y());
  }

  static SLPoint ToSLPoint(
      const double s, const double l, 
      const double dl = 0.0, const double ddl = 0.0) {
    SLPoint sl;
    sl.set_s(s);
    sl.set_l(l);
    sl.set_dl(dl);
    sl.set_ddl(ddl);
    return sl;
  }

  static SpeedPoint ToSpeedPoint(
      const double s, const double t,
      const double v = 0, const double a = 0,const double da = 0) {
    SpeedPoint speed_point;
    speed_point.set_s(s);
    speed_point.set_t(t);
    speed_point.set_v(v);
    speed_point.set_a(a);
    speed_point.set_da(da);
    return speed_point;
  }

  static PathPoint ToPathPoint(
      const double x, const double y,
      const double z = 0, const double s = 0,
      const double theta = 0, const double kappa = 0,
      const double dkappa = 0, const double ddkappa = 0) {
    PathPoint path_point;
    path_point.set_x(x);
    path_point.set_y(y);
    path_point.set_z(z);
    path_point.set_s(s);
    path_point.set_theta(theta);
    path_point.set_kappa(kappa);
    path_point.set_dkappa(dkappa);
    path_point.set_ddkappa(ddkappa);

    return path_point;
  }
};

} //namespace pnc
} //namespace xju
