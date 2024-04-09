/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/tasks/deciders/speed_decider/feasible_region.h"

#include <cmath>

#include "planning/common/planning_gflags/planning_gflags.h"

namespace {
static constexpr double kDoubleEpsilon = 1e-6;
}  // namespace

namespace xju {
namespace planning {

FeasibleRegion::FeasibleRegion(const std::array<double, 3>& init_s) {
  init_s_ = init_s;
  double init_v = init_s[1];
  if (init_v < 1e-3) init_v = 0.0;
  v_max_ = std::fmax(v_max_, init_v);

  const double max_acceleration = FLAGS_longitudinal_acceleration_upper_bound;
  const double min_acceleration = FLAGS_longitudinal_acceleration_lower_bound;

  t_at_zero_speed_ = -init_v / (min_acceleration - kDoubleEpsilon);
  // ADEBUG << "t_at_zero_speed_" << t_at_zero_speed_;
  s_at_zero_speed_ = init_s[0] + (-init_v * init_v) / (2.0 * min_acceleration);
  // ADEBUG << "s_at_zero_speed_" << s_at_zero_speed_;
  t_at_max_speed_ = (v_max_ - init_v) / (max_acceleration + kDoubleEpsilon);
  s_at_max_speed_ =
      init_s_[0] + (v_max_ * v_max_ - init_v * init_v) / (2 * max_acceleration);
}

double FeasibleRegion::SUpper(const double t) const {
  if (t < t_at_max_speed_) {
    return init_s_[0] + init_s_[1] * t +
           0.5 * FLAGS_longitudinal_acceleration_upper_bound * t * t;
  }
  return s_at_max_speed_ + v_max_ * (t - t_at_max_speed_);
}

double FeasibleRegion::SLower(const double t) const {
  if (t < t_at_zero_speed_) {
    return init_s_[0] + init_s_[1] * t +
           0.5 * FLAGS_longitudinal_acceleration_lower_bound * t * t;
  }
  return s_at_zero_speed_;
}

double FeasibleRegion::VUpper(const double t) const {
  return t < t_at_max_speed_
             ? init_s_[1] + FLAGS_longitudinal_acceleration_upper_bound * t
             : v_max_;
}

double FeasibleRegion::VLower(const double t) const {
  return t < t_at_zero_speed_
             ? init_s_[1] + FLAGS_longitudinal_acceleration_lower_bound * t
             : 0.0;
}

double FeasibleRegion::TLower(const double s) const {
  double delta_s = s - init_s_[0];
  double v = init_s_[1];
  double a = FLAGS_longitudinal_acceleration_upper_bound;
  double t = (std::sqrt(v * v + 2.0 * a * delta_s) - v) / a;
  return t;
}
}  // namespace planning
}  // namespace xju
