/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/golden_search.h"
#include <cmath>

namespace xju {
namespace pnc {
double GoldenSearch(const std::function<double(double)> &func,
                    const double lower_bound, const double upper_bound,
                    const double tol) {
  static constexpr double gr = 1.618033989;

  double a = lower_bound;
  double b = upper_bound;

  double t = (b - a) / gr;
  double c = b - t;
  double d = a + t;

  while (std::abs(c - d) > tol) {
    if (func(c) < func(d)) {
      b = d;
    } else {
      a = c;
    }
    t = (b - a) / gr;
    c = b - t;
    d = a + t;
  }
  return (a + b) * 0.5;
}

}  // namespace pnc
}  // namespace xju