/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/curve_math.h"

#include "gtest/gtest.h"

#include "common/math/math_utils.h"
#include "common/logger/logger.h"

namespace xju {
namespace pnc {

TEST(CurveMathTest, ComputeCurvature1) {
  // test x(t) = t, y(t) = t; 
  double dx = 1.0;
  double ddx = 0.0;
  double dddx = 0.0;
  double dy = 1.0;
  double ddy = 0.0;
  double dddy = 0.0;
  double kappa = CurveMath::ComputeCurvature(dx, ddx, dy, ddy);
  double dkappa = CurveMath::ComputeCurvatureDerivative(
      dx, ddx, dddx, dy, ddy, dddy);
  EXPECT_NEAR(kappa, 0.0, kMathEpsilon);
  EXPECT_NEAR(dkappa, 0.0, kMathEpsilon);

  // test x(t) = cos(t), y(t) = sin(t)
  double t = M_PI_4;
  dx = -std::sin(t);
  ddx = -std::cos(t);
  dddx = std::sin(t);
  dy = std::cos(t);
  ddy = -std::sin(t);
  dddy = -std::cos(t);
  kappa = CurveMath::ComputeCurvature(dx, ddx, dy, ddy);
  dkappa = CurveMath::ComputeCurvatureDerivative(
      dx, ddx, dddx, dy, ddy, dddy);
  EXPECT_NEAR(kappa, 1.0, kMathEpsilon);
  EXPECT_NEAR(dkappa, 0.0, kMathEpsilon);
}

TEST(CurveMathTest, ComputeCurvature2) {
  // test y(x) = x; 
  double dy = 1.0;
  double ddy = 0.0;
  double dddy = 0.0;
  double kappa = CurveMath::ComputeCurvature(dy, ddy);
  double dkappa = CurveMath::ComputeCurvatureDerivative(dy, ddy, dddy);
  EXPECT_NEAR(kappa, 0.0, kMathEpsilon);
  EXPECT_NEAR(dkappa, 0.0, kMathEpsilon);

  // test y(x) = sqrt(1 - x^2)
  auto func_y = [](const double x) {
    return std::sqrt(1  - x * x);
  };
  auto func_dy = [](const double x) {
    return -x / std::sqrt(1 - x * x);
  };
  auto func_ddy = [&func_y](const double x) {
    return -(func_y(x) + x * x / func_y(x)) / (1  - x * x);
  };

  auto func_dddy = [&func_y](const double x) {
    return -((4 * x * x * x - x) / func_y(x) + 4 * x * func_y(x)) / (1 - x * x) / (1 - x * x);
  };

  double x = std::cos(M_PI_4);
  dy = func_dy(x);
  ddy = func_ddy(x);
  dddy = func_dddy(x);
  kappa = CurveMath::ComputeCurvature(dy, ddy);
  dkappa = CurveMath::ComputeCurvatureDerivative(dy, ddy, dddy);
  EXPECT_NEAR(kappa, -1.0, kMathEpsilon);
  EXPECT_NEAR(dkappa, 0.0, kMathEpsilon);
}

TEST(CurveMathTest, ComputeCurvature3) {
  // test x(t) = cos(t), y(t) = sin(t)
  double t1 = M_PI_4 - M_PI_4 / 2.0;
  double t2 = M_PI_4;
  double t3 = M_PI_4 + M_PI_4 / 2.0;

  Vec2d pt1(std::cos(t1), std::sin(t1));
  Vec2d pt2(std::cos(t2), std::sin(t2));
  Vec2d pt3(std::cos(t3), std::sin(t3));
  
  double kappa = CurveMath::ComputeCurvature(pt1, pt2, pt3);
  EXPECT_NEAR(kappa, 1.0, kMathEpsilon);

  kappa = CurveMath::ComputeCurvature(pt3, pt2, pt1);
  EXPECT_NEAR(kappa, -1.0, kMathEpsilon);

  kappa = CurveMath::ComputeCurvature(
      Vec2d(0.0, 1.0), Vec2d(0.0, 2.0), Vec2d(0.0, 3.0));
  EXPECT_NEAR(kappa, 0.0, kMathEpsilon);
}

} // namespace pnc
} // namespace xju