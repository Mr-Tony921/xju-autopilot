/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/tasks/deciders/speed_decider/kinematics_model.h"

#include <cmath>

namespace xju {
namespace planning {

void ConstantAccelerationModelT::Init(float s0, float v0, float a) {
  if (std::fabs(a) < 1e-3) {
    a = 0.0f;
  }
  if (a < -1e-3) {
    this->brake_time_ = std::fabs(v0 / a);
    this->brake_dis_ = v0 * v0 / (-2.0f * a);
    this->vmax_time_ = std::numeric_limits<float>::max();  
    this->vmax_dis_ = std::numeric_limits<float>::max();
  } else {
    this->brake_time_ = std::numeric_limits<float>::max();
    this->brake_dis_ = std::numeric_limits<float>::max();
    this->vmax_time_ = (v_max_ - v0) / a; 
    this->vmax_dis_ = (v_max_ * v_max_ - v0 * v0) / (2.0f * a);
  }
    this->s0_ = s0;
    this->v0_ = v0;
    this->a_ = a;
}

float ConstantAccelerationModelT::EvaluateTranslation(float t) const {
  if (a_ < 1e-3) {
    if (t > this->brake_time_) {
        return this->s0_ + this->brake_dis_;
    } else {
        return this->s0_ + this->v0_ * t + 0.5f * this->a_ * t * t;
    }
  } else {
    if (t > this->vmax_time_) {
        return this->s0_ + this->vmax_dis_+ this->v_max_ * (t - this->v_max_);
    } else {
        return this->s0_ + this->v0_ * t + 0.5f * this->a_ * t * t;
    }
  }

}

float ConstantAccelerationModelT::EvaluateVelocity(float t) const {
    if (t > this->brake_time_) {
        return 0.0f;
    } else {
        return this->v0_ + this->a_ * t;
    }
}

float ConstantAccelerationModelT::StartDistance() const { return s0_; }

float ConstantAccelerationModelT::Acceleration() const { return a_; }

float ConstantAccelerationModelT::StartVelocity() const { return v0_; }
} // namespace planning
} // namespace xju
