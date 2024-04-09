/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <array>

namespace xju {
namespace planning {
class FeasibleRegion {
 public:
  explicit FeasibleRegion(const std::array<double, 3>& init_s);

  double SUpper(const double t) const;

  double SLower(const double t) const;

  double VUpper(const double t) const;

  double VLower(const double t) const;

  double TLower(const double s) const;

 private:
  std::array<double, 3> init_s_;
  
  double v_max_ = 40.0;

  double t_at_zero_speed_;
  
  double t_at_max_speed_;

  double s_at_zero_speed_;
  double s_at_max_speed_;
};
} // namespace planning
} // namespace xju
