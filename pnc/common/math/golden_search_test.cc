/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/golden_search.h"

#include <cmath>

#include "gtest/gtest.h"

namespace xju {
namespace pnc {
namespace {
double LinearFunc(double x) { return 2.0 * x; }
double LinearFunc2(double x) { return -2.0 * x; }
double SquareFunc(double x) { return x * x; }
double CubicFunc(double x) { return (x - 1.0) * (x - 2.0) * (x - 3.0); }
double SinFunc(double x) { return std::sin(x); }
}  // namespace

TEST(GoldenSearchTest, test) {
  double linear_argmin = GoldenSearch(LinearFunc, 0.0, 1.0, 1e-6);
  EXPECT_NEAR(linear_argmin, 0.0, 1e-5);
  linear_argmin = GoldenSearch(LinearFunc2, 0.0, 1.0, 1e-6);
  EXPECT_NEAR(linear_argmin, 1.0, 1e-5);

  double square_argmin = GoldenSearch(SquareFunc, -0.1, 2.0, 1e-6);
  EXPECT_NEAR(square_argmin, 0.0, 1e-5);

  double cubic_argmin = GoldenSearch(CubicFunc, 0.0, 1.5, 1e-6);
  EXPECT_NEAR(cubic_argmin, 0.0, 1e-5);
  cubic_argmin = GoldenSearch(CubicFunc, 1.0, 1.8, 1e-6);
  EXPECT_NEAR(cubic_argmin, 1.0, 1e-5);
  cubic_argmin = GoldenSearch(CubicFunc, 2.0, 3.0, 1e-6);
  EXPECT_NEAR(cubic_argmin, 2.0 + 1.0 / std::sqrt(3.0), 1e-5);

  double sin_argmin = GoldenSearch(SinFunc, 0.0, 2 * M_PI, 1e-6);
  EXPECT_NEAR(sin_argmin, 1.5 * M_PI, 1e-5);
}

}  // namespace pnc
}  // namespace xju
