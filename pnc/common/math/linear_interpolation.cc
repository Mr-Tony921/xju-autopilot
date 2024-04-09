/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/linear_interpolation.h"
namespace xju {
namespace pnc {

double AngleSlerp(const double a0, const double t0, const double a1, 
                  const double t1, const double t) {
  if (std::fabs(t1 - t0) <= kMathEpsilon) {
    ADEBUG << "input time difference is too small";
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle(a);
}

double AngleSlerp(const double a0, const double a1, const double t) {
  if (std::fabs(t) <= kMathEpsilon) {
    ADEBUG << "input time difference is too small";
    return NormalizeAngle(a0);
  }
  const double a0_n = NormalizeAngle(a0);
  const double a1_n = NormalizeAngle(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double a = a0_n + d * t;
  return NormalizeAngle(a);
}

SLPoint Interpolate(const SLPoint& p0, 
                    const SLPoint& p1, const double w) {
  CHECK_GE(w, 0.0);
  SLPoint p;
  p.set_s((1 - w) * p0.s() + w * p1.s());
  p.set_l((1 - w) * p0.l() + w * p1.l());
  p.set_dl((1 - w) * p0.dl() + w * p1.dl());
  p.set_ddl((1 - w) * p0.ddl() + w * p1.ddl());
  return p;
}

PathPoint InterpolateByRatio(
    const PathPoint& p0, const PathPoint& p1, const double r) {
  PathPoint pt;
  pt.set_x((1 - r) * p0.x() + r * p1.x());
  pt.set_y((1 - r) * p0.y() + r * p1.y());
  pt.set_theta(AngleSlerp(p0.theta(), p1.theta(), r));
  pt.set_kappa((1 - r) * p0.kappa() + r * p1.kappa());
  pt.set_dkappa((1 - r) * p0.dkappa() + r * p1.dkappa());
  pt.set_ddkappa((1 - r) * p0.ddkappa() + r * p1.ddkappa());
  pt.set_s(p0.s() + r * (p1.s() - p0.s()));
  return pt;
}

PathPoint Interpolate(const PathPoint& p0, 
                      const PathPoint& p1, const double s) {
  double s0 = p0.s();
  double s1 = p1.s();
  double r = (s - s0) / (s1 - s0);

  PathPoint pt;
  pt.set_x((1 - r) * p0.x() + r * p1.x());
  pt.set_y((1 - r) * p0.y() + r * p1.y());
  pt.set_theta(AngleSlerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s));
  pt.set_kappa((1 - r) * p0.kappa() + r * p1.kappa());
  pt.set_dkappa((1 - r) * p0.dkappa() + r * p1.dkappa());
  pt.set_ddkappa((1 - r) * p0.ddkappa() + r * p1.ddkappa());
  pt.set_s(s);
  return pt;
}

TrajectoryPoint Interpolate(const TrajectoryPoint& p0, 
                            const TrajectoryPoint& p1, const double t) {
  if (!p0.has_path_point() || !p1.has_path_point()) {
    TrajectoryPoint pt;
    pt.mutable_path_point()->CopyFrom(PathPoint());
    return pt;
  }
  const PathPoint pp0 = p0.path_point();
  const PathPoint pp1 = p1.path_point();
  double t0 = p0.relative_time();
  double t1 = p1.relative_time();

  TrajectoryPoint tp;
  tp.set_v(lerp(p0.v(), t0, p1.v(), t1, t));
  tp.set_a(lerp(p0.a(), t0, p1.a(), t1, t));
  if (!p0.has_da() || !p1.has_da()) {
    tp.set_da(0.0);
  } else {
    tp.set_da(lerp(p0.da(), t0, p1.da(), t1, t));
  }
  tp.set_relative_time(t);

  PathPoint* path_point = tp.mutable_path_point();
  path_point->set_x(lerp(pp0.x(), t0, pp1.x(), t1, t));
  path_point->set_y(lerp(pp0.y(), t0, pp1.y(), t1, t));
  path_point->set_s(lerp(pp0.s(), t0, pp1.s(), t1, t));
  path_point->set_theta(AngleSlerp(pp0.theta(), t0, pp1.theta(), t1, t));
  path_point->set_kappa(lerp(pp0.kappa(), t0, pp1.kappa(), t1, t));
  path_point->set_dkappa(lerp(pp0.dkappa(), t0, pp1.dkappa(), t1, t));
  path_point->set_ddkappa(lerp(pp0.ddkappa(), t0, pp1.ddkappa(), t1, t));

  return tp;
}

} // namespace pnc
} // namespace xju
