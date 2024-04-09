/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/filter/second_order_lowpass_filter.h"


namespace xju {
namespace pnc {

void SecondOrderLowPassFilter::Init(const double sample_time,
                                   const double cutoff_freq,
                                   const double dead_zone) {
  if (sample_time <= 0.0 || cutoff_freq <= 0.0) {
    return;
  }
  
  sample_time_ = sample_time;
  cutoff_freq_ = cutoff_freq;
  dead_zone_ = dead_zone;

  denominators_.clear();
  numerators_.clear();
  denominators_.reserve(3);
  numerators_.reserve(3);

  double wa = 2.0 * M_PI * cutoff_freq;  // Analog frequency in rad/s
  double alpha = wa * sample_time / 2.0;          // tan(Wd/2), Wd is discrete frequency
  double alpha_sqr = alpha * alpha;
  double tmp_term = std::sqrt(2.0) * alpha + alpha_sqr;
  double gain = alpha_sqr / (1.0 + tmp_term);

  denominators_.push_back(1.0);
  denominators_.push_back(2.0 * (alpha_sqr - 1.0) / (1.0 + tmp_term));
  denominators_.push_back((1.0 - std::sqrt(2.0) * alpha + alpha_sqr) /
                          (1.0 + tmp_term));

  numerators_.push_back(gain);
  numerators_.push_back(2.0 * gain);
  numerators_.push_back(gain);
  
  digital_filter_.Init(numerators_, denominators_, dead_zone);
}

} // namespace pnc
} // namespace utils
