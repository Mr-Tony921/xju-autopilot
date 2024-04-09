/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "common/filter/digital_filter.h"
#include "common/logger/logger.h"

namespace xju {
namespace pnc {

class FirstOrderLowPassFilter{
 public:
  FirstOrderLowPassFilter() = default;
  ~FirstOrderLowPassFilter() = default;
  FirstOrderLowPassFilter(const double sample_time,
                          const double cutoff_freq,
                          const double dead_zone = 0.0) {
    Init(sample_time, cutoff_freq, dead_zone);
  }

  void Init(const double sample_time,
            const double cutoff_freq,
            const double dead_zone = 0.0);

  void InitTimeDomain(
      const double sample_time, const double settling_time,
      const double dead_time = 0.0, const double dead_zone = 0.0);

  double Filter(const double x_insert) { return digital_filter_.Filter(x_insert); }

  std::vector<double> numerators() const { return numerators_; }

  std::vector<double> denominators() const { return denominators_; }
  
  double sample_time() const { return sample_time_; }

  double cutoff_freq() const { return cutoff_freq_; }

  double dead_zone() const { return dead_zone_; }

 private:
  // Coefficients with x values
  std::vector<double> numerators_;
  // Coefficients with y values
  std::vector<double> denominators_;

  double sample_time_ = 0.0;
  double cutoff_freq_ = 0.0;
  double dead_zone_ = 0.0;

  DigitalFilter digital_filter_;
};

} // namespace pnc
} // namespace xju