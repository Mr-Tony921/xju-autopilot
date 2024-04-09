/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include <cmath>
#include <vector>

#include"common/math/polynomial_x_order.h"

namespace xju {
namespace pnc {


const int MAX_ORDER = 4;

void PolynomialXOrder::Init(const std::vector<double> &coef) {
  coef_at_order_.clear();
  coef_at_order_.push_back(coef);
  for (int order = 1; order < MAX_ORDER; order++) {
    std::vector<double> &coef_prev = coef_at_order_[order - 1];
    std::vector<double> coef_now;
    for (int i = 1; (unsigned int)i < coef_prev.size(); i++) {
      coef_now.push_back(coef_prev[i] * i);
    }
    coef_at_order_.push_back(coef_now);
  }
}

double PolynomialXOrder::Eval(int order, double x) const {
  if (order >= MAX_ORDER) {
    return 0;
  }
  const std::vector<double> &coef = coef_at_order_.at(order);
  double result = 0;
  double x_norm_order = std::pow(x_norm_, order);
  for (int i = coef.size() - 1; i >= 1; i--) {
    result = (x / x_norm_) * (result + coef[i]);
  }
  if (coef.size() > 0) {
    result += coef[0];
  }

  return result / x_norm_order;
}

const std::vector<double>& PolynomialXOrder::Coef(int order) const {
  if (order >= MAX_ORDER) {
    order = 0;
  }
  return coef_at_order_[order];
};

const std::array<double, 3>& PolynomialXOrder::start_state() const {
  return start_state_;
}

const std::pair<std::array<double, 3>, double>& PolynomialXOrder::end_state() const {
  return end_state_;
}

const std::string& PolynomialXOrder::label() const {
  return label_;
}

void PolynomialXOrder::set_start_state(const std::array<double, 3>& start_state) {
  start_state_ = start_state;
}

void PolynomialXOrder::set_end_state(const std::pair<std::array<double, 3>, double>& end_state) {
  end_state_ = end_state;  
}

void PolynomialXOrder::set_label(const std::string& label) {
  label_ = label;
}

} // namespace pnc  
} // namespace xju

