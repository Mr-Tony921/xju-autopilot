/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <deque>
#include <vector>
#include <cmath>

#include "common/logger/logger.h"

/**
 * @file
 * @brief Defines Universal DigitalFilter class
 *                a_n * z^n + a_n-1 * z^n-1 + ... + a_0
 *        G(z) = ---------------------------------------            
 *                b_n * z^n + b_n-1 * z^n-1 + ... + b_0
 */

namespace {
  const double kDoubleEpsilon = 1.0e-6;
} // namespace

namespace xju {
namespace pnc {

class DigitalFilter {
 public:
  DigitalFilter() = default;
  DigitalFilter(const std::vector<double>& numerators,
                const std::vector<double>& denominators,
                const double dead_zone = 0.0) {
    Init(numerators, denominators, dead_zone);
  }

  ~DigitalFilter() = default;
  void Init(const std::vector<double>& numerators,
            const std::vector<double>& denominators,
            const double dead_zone = 0.0);
  
  double Filter(const double x_insert);

  void Reset();

  void set_numerators(const std::vector<double>& numerators) {
    numerators_ = numerators;
    x_values_.resize(numerators_.size(), 0.0);
  }

  void set_denominators(const std::vector<double>& denominators) {
    denominators_ = denominators;
    y_values_.resize(denominators_.size(), 0.0);
  }

  void set_coefficients(const std::vector<double> &denominators,
                        const std::vector<double> &numerators) {
    set_denominators(denominators);
    set_numerators(numerators);
  }

  void set_dead_zone(const double dead_zone) { dead_zone_ = std::fabs(dead_zone); }

  std::vector<double> numerators() const { return numerators_; }

  std::vector<double> denominators() const { return denominators_; }

  double dead_zone() const { return dead_zone_; }

 private:
  double Compute(const std::deque<double>& values,
                 const std::vector<double>& coefficients,
                 const std::size_t coeff_start,
                 const std::size_t coeff_end);
  
  double UpdateLast(const double input);

 private:
  // Front is latest, back is oldest.
  std::deque<double> x_values_;
  // Front is latest, back is oldest.
  std::deque<double> y_values_;
  // Coefficients with x values
  std::vector<double> numerators_;
  // Coefficients with y values
  std::vector<double> denominators_;
  // threshold of updating last-filtered value
  double dead_zone_ = 0.0;
  // last-filtered value
  double last_ = 0.0;
};

} // namespace pnc
} // namespace xju

