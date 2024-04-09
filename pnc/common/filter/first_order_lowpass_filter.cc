/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/filter/first_order_lowpass_filter.h"

namespace xju {
namespace pnc {

void FirstOrderLowPassFilter::Init(const double sample_time,
                                   const double cutoff_freq,
                                   const double dead_zone) {
  if (sample_time <= 0.0 || cutoff_freq <= 0.0) {
    return;
  }
    
  sample_time_ = sample_time;
  cutoff_freq_ = cutoff_freq;
  dead_zone_ = dead_zone;

  numerators_.clear();
  denominators_.clear();
  numerators_.reserve(2);
  denominators_.reserve(2);
   
  double wn = 2 * M_PI * cutoff_freq;

  numerators_.insert(numerators_.end(), 2, wn * sample_time);
  denominators_.push_back(wn * sample_time + 2);
  denominators_.push_back(wn * sample_time - 2);
  
  digital_filter_.Init(numerators_, denominators_, dead_zone);
}

void FirstOrderLowPassFilter::InitTimeDomain(const double sample_time,
                      const double settling_time,
                      const double dead_zone,
                      const double dead_time) {
    if (sample_time <= 0.0 || settling_time < 0.0 || dead_time < 0.0) {
    return;
  } 
  
  const size_t k_d = static_cast<size_t>(dead_time / sample_time);
  double a_term = 0.0;

  numerators_.clear();
  denominators_.clear();
  numerators_.reserve(k_d + 1);
  denominators_.reserve(2);

  if (settling_time == 0.0) {
    a_term = 0.0;
  } else {
    a_term = exp(-1 * sample_time / settling_time);
  }
  
  numerators_.insert(numerators_.end(), k_d, 0.0);
  numerators_.push_back(1 - a_term);
  denominators_.push_back(1.0);
  denominators_.push_back(-a_term);

  digital_filter_.Init(numerators_, denominators_, dead_zone);
}

} // namespace pnc
} // namespace xju