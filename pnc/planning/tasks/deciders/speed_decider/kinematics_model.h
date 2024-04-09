/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <limits>

namespace xju {
namespace planning {
class ConstantAccelerationModelT {
 public:
  void Init(float s0, float v0, float a = 0);

  float EvaluateTranslation(float t) const;

  float EvaluateVelocity(float t) const;

  float StartDistance() const;

  float StartVelocity() const;

  float Acceleration() const;

 private:
  float a_ = 0;

  float v0_ = 0;

  float s0_ = 0;
  
  float v_max_ = 35.0;

  float brake_dis_ = std::numeric_limits<float>::max();

  float brake_time_ = std::numeric_limits<float>::max();

  float vmax_time_ = std::numeric_limits<float>::max();

  float vmax_dis_ = std::numeric_limits<float>::max();
};


} // namespace planning 
} // namespace xju
