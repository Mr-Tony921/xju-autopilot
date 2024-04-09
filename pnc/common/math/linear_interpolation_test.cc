/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/linear_interpolation.h"
#include "gtest/gtest.h"
namespace xju {
namespace pnc {
  //AngleSlerp
TEST(linear_interpolation_test, AngleSlerp_1){
  const double a0 = 5;
  const double a1 = 8;
  const double t0 = 1;
  const double t1 = 1;
  const double t = 1.5;
  double res = AngleSlerp(a0, t0, a1, t1, t);
  EXPECT_NEAR(res , NormalizeAngle(5), kMathEpsilon);
}

TEST(linear_interpolation_test, AngleSlerp_2){
  const double a0 = 1;
  const double a1 = 2;
  const double t0 = 1;
  const double t1 = 2;
  const double t = 1.5;
  double res = AngleSlerp(a0, t0, a1, t1, t);
  EXPECT_NEAR(res , 1.5, kMathEpsilon);
}

TEST(linear_interpolation_test, AngleSlerp_3){
  const double a0 = -2;
  const double a1 = M_PI - 1;
  const double t0 = 1;
  const double t1 = 2;
  const double t = 1.5;
  double res = AngleSlerp(a0, t0, a1, t1, t);
  EXPECT_NEAR(res , -1.5 - 0.5*M_PI, kMathEpsilon);
}

//AngleSlerp,overload funciton
TEST(linear_interpolation_test, AngleSlerp_overload1_1){
  const double a0 = -2;
  const double a1 = M_PI - 1;
  const double t = 1.5;
  double res = AngleSlerp(a0, a1, t);
  EXPECT_NEAR(res , 0.5*M_PI - 0.5, kMathEpsilon);
}

  //Interpolate
TEST(linear_interpolation_test, Interpolate_1){
  SLPoint p0, p1;
  p0.set_s(20);
  p0.set_l(0.5);
  p0.set_dl(1);
  p0.set_ddl(0.5);
  p1.set_s(22);
  p1.set_l(0.3);
  p1.set_dl(0.5);
  p1.set_ddl(0.7);
  double w = 0;
  SLPoint p = Interpolate(p0, p1, w);
  EXPECT_NEAR(p.s() , 20, kMathEpsilon);
  EXPECT_NEAR(p.l() , 0.5, kMathEpsilon);
  EXPECT_NEAR(p.dl() , 1, kMathEpsilon);
  EXPECT_NEAR(p.ddl() , 0.5, kMathEpsilon);
}

TEST(linear_interpolation_test, Interpolate_2){
  SLPoint p0, p1;
  p0.set_s(20);
  p0.set_l(0.5);
  p0.set_dl(1);
  p0.set_ddl(0.5);
  p1.set_s(22);
  p1.set_l(0.3);
  p1.set_dl(0.5);
  p1.set_ddl(0.7);
  double w = 0.3;
  SLPoint p = Interpolate(p0, p1, w);
  EXPECT_NEAR(p.s() , 20.6, kMathEpsilon);
  EXPECT_NEAR(p.l() , 0.44, kMathEpsilon);
  EXPECT_NEAR(p.dl() , 0.85, kMathEpsilon);
  EXPECT_NEAR(p.ddl() , 0.56, kMathEpsilon);
}

//InterpolateByRatio
TEST(linear_interpolation_test, InterpolateByRatio_1){
  PathPoint p0, p1;
  p0.set_x(10);
  p0.set_y(8);
  p0.set_theta(2);
  p0.set_kappa(0.01);
  p0.set_dkappa(0.05);
  p0.set_s(20);
  p1.set_x(12);
  p1.set_y(7);
  p1.set_theta(2.2);
  p1.set_kappa(0.012);
  p1.set_dkappa(0.07);
  p1.set_s(22);
  PathPoint p = InterpolateByRatio(p0, p1, 0.3);
  EXPECT_NEAR(p.x() , 10.6, kMathEpsilon);
  EXPECT_NEAR(p.y() , 7.7, kMathEpsilon);
  EXPECT_NEAR(p.theta() , 2.06, kMathEpsilon);
  EXPECT_NEAR(p.kappa() , 0.0106, kMathEpsilon);
  EXPECT_NEAR(p.dkappa() , 0.056, kMathEpsilon);
  EXPECT_NEAR(p.s() , 20.6, kMathEpsilon);
}

//Interpolate
TEST(linear_interpolation_test, Interpolate_overload1_1) {
  PathPoint p0, p1;
  p0.set_x(10);
  p0.set_y(8);
  p0.set_theta(2);
  p0.set_kappa(0.01);
  p0.set_dkappa(0.05);
  p0.set_ddkappa(0.04);
  p0.set_s(20);

  p1.set_x(12);
  p1.set_y(7);
  p1.set_theta(2.2);
  p1.set_kappa(0.012);
  p1.set_dkappa(0.07);
  p1.set_ddkappa(0.03);
  p1.set_s(22);
  PathPoint p = Interpolate(p0, p1, 21);
  EXPECT_NEAR(p.x() , 11, kMathEpsilon);
  EXPECT_NEAR(p.y() , 7.5, kMathEpsilon);
  EXPECT_NEAR(p.theta() , 2.1, kMathEpsilon);
  EXPECT_NEAR(p.kappa() , 0.011, kMathEpsilon);
  EXPECT_NEAR(p.dkappa() , 0.06, kMathEpsilon);
  EXPECT_NEAR(p.ddkappa() , 0.035, kMathEpsilon);
  EXPECT_NEAR(p.s() , 21, kMathEpsilon);
}

//Interpolate,if logic tested by log.
TEST(linear_interpolation_test, Interpolate_overload2_2) {
  TrajectoryPoint p0, p1;
  p0.mutable_path_point()->set_x(10);
  p0.mutable_path_point()->set_y(8);
  p0.mutable_path_point()->set_s(20);
  p0.mutable_path_point()->set_theta(2);
  p0.mutable_path_point()->set_kappa(0.01);
  p0.mutable_path_point()->set_dkappa(0.05);
  p0.mutable_path_point()->set_ddkappa(0.03);
  p0.set_relative_time(1);
  p0.set_v(3);
  p0.set_a(1);
  p0.set_da(0.4);

  p1.mutable_path_point()->set_x(12);
  p1.mutable_path_point()->set_y(7);
  p1.mutable_path_point()->set_s(22);
  p1.mutable_path_point()->set_theta(2.2);
  p1.mutable_path_point()->set_kappa(0.012);
  p1.mutable_path_point()->set_dkappa(0.07);
  p1.mutable_path_point()->set_ddkappa(0.04);
  p1.set_relative_time(3);
  p1.set_v(4);
  p1.set_a(1.2);
  p1.set_da(0.6);

  TrajectoryPoint p = Interpolate(p0, p1, 2);
  EXPECT_NEAR(p.mutable_path_point()->x() , 11, kMathEpsilon);
  EXPECT_NEAR(p.mutable_path_point()->y() , 7.5, kMathEpsilon);
  EXPECT_NEAR(p.mutable_path_point()->s() , 21, kMathEpsilon);
  EXPECT_NEAR(p.mutable_path_point()->theta() , 2.1, kMathEpsilon);
  EXPECT_NEAR(p.mutable_path_point()->kappa() , 0.011, kMathEpsilon);
  EXPECT_NEAR(p.mutable_path_point()->dkappa() , 0.06, kMathEpsilon);
  EXPECT_NEAR(p.mutable_path_point()->ddkappa() , 0.035, kMathEpsilon);
  EXPECT_NEAR(p.v() , 3.5, kMathEpsilon);
  EXPECT_NEAR(p.a() , 1.1, kMathEpsilon);
  EXPECT_NEAR(p.da() , 0.5, kMathEpsilon);
}


} // namespace pnc
} // namespace xju
