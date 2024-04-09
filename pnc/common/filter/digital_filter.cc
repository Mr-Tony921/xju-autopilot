/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/filter/digital_filter.h"

namespace xju {
namespace pnc {

const double kDoubleEpsilon = 1.0e-6;

void DigitalFilter::Init(const std::vector<double>& numerators,
                         const std::vector<double>& denominators,
                         const double dead_zone) {
  set_dead_zone(dead_zone);
  set_numerators(numerators);
  set_denominators(denominators);
}

double DigitalFilter::Filter(const double x_insert) {
  if (numerators_.empty() || denominators_.empty()) {
    return 0.0;
  }

  x_values_.pop_back();
  x_values_.push_front(x_insert);
  const double xside =
      Compute(x_values_, numerators_, 0, numerators_.size() - 1);

  y_values_.pop_back();
  const double yside =
      Compute(y_values_, denominators_, 1, denominators_.size() - 1);

  double y_insert = 0.0;
  if (std::fabs(denominators_.front()) > kDoubleEpsilon) {
    y_insert = (xside - yside) / denominators_.front();
  }
  y_values_.push_front(y_insert);

  return UpdateLast(y_insert);
}

void DigitalFilter::Reset() {
  std::fill(x_values_.begin(), x_values_.end(), 0.0);
  std::fill(y_values_.begin(), y_values_.end(), 0.0);
}

double DigitalFilter::Compute(const std::deque<double>& values,
                              const std::vector<double>& coefficients,
                              const std::size_t coeff_start,
                              const std::size_t coeff_end) {
  if (coeff_end >= coefficients.size() || coeff_start > coeff_end ||
      (coeff_end - coeff_start + 1 != values.size())) {
    return 0.0;
  }
  double sum = 0.0;
  auto i = coeff_start;
  for (const double& value : values) {
    sum += value * coefficients.at(i);
    ++i;
  }
  return sum;
}

double DigitalFilter::UpdateLast(const double input) {
  const double diff = std::fabs(input - last_);
  if (diff < dead_zone_) {
    return last_;
  }
  last_ = input;
  return input;
}

}  // namespace pnc
}  // namespace xju