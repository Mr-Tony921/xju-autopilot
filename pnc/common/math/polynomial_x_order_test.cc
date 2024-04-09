/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/polynomial_x_order.h"

#include <cmath>
#include "gtest/gtest.h"

namespace xju {
namespace pnc {

TEST(PolynomialXOrderTest, Init) {

    PolynomialXOrder poly;
    poly.Init({0.0, 0.0, 0.0, 0.0, 0.0});
}

TEST(PolynomialXOrderTest, Order3) {
    double a = 1;
    double b = 2;
    double c = 3;
    std::vector<double> T = {0, 1.1, 2.3, 4.1, 5.7};
    for (auto t : T) {
        std::vector<double> coef = {0, c, b, a};

        double r = 0;
        double gt = 0;
        PolynomialXOrder poly;
        poly.Init(coef);
        r = poly.Eval(0, t);
        gt = a * pow(t, 3) + b * pow(t, 2) + c * t;
        EXPECT_NEAR(r, gt, 1e-6);
        r = poly.Eval(1, t);
        gt = 3 * a * pow(t, 2) + 2 * b * t + c;
        EXPECT_NEAR(r, gt, 1e-6);
        gt = 6 * a * t + 2 * b;
        r = poly.Eval(2, t);
        EXPECT_NEAR(r, gt, 1e-6);
    }
}
} // namespace pnc
} // namespace xju
