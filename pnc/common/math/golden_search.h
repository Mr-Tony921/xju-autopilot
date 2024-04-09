/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <functional>

namespace xju {
namespace pnc {

double GoldenSearch(const std::function<double(double)> &func,
                    const double lower_bound, const double upper_bound,
                    const double tol = 1e-6);

}  // namespace pnc
}  // namespace xju
