/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#pragma once

#include <vector>
#include <array>
#include <string>

namespace xju {
namespace pnc {

class PolynomialXOrder {
 public:
  //coef in reverse order 
  PolynomialXOrder() : x_norm_(1.0) {}
  ~PolynomialXOrder() = default;
  void Init(const std::vector<double> &coef);
  double Eval(int order, double x) const;
  
  const std::vector<double>& Coef(int order = 0) const;

  const std::array<double, 3>& start_state() const;

  const std::pair<std::array<double, 3>, double>& end_state() const;

  const std::string& label() const;

  void set_start_state(const std::array<double, 3>& start_state);

  void set_end_state(const std::pair<std::array<double, 3>, double>& end_state);

  void set_label(const std::string& label);

 private:
  double x_norm_;
  std::vector<std::vector<double>> coef_at_order_;
  std::array<double, 3> start_state_{0.0, 0.0, 0.0};
  std::pair<std::array<double, 3>, double> end_state_;
  std::string label_;
}; 
} // namespace pnc  
} // namespace xju
