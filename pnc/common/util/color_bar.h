/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "common/design_pattern/singleton.h"

namespace xju {
namespace pnc {

class ColorBar {
 public:
  // right: blue
  // left: red
  void GetRGBRate(const float rate, float* r_rate, float* g_rate,
                  float* b_rate) {
    float r, g, b;
    int len = rate * size_;
    if (len < 0) {
      len = 0;
    }
    if (len >= size_) {
      len = size_;
    }
    *r_rate = R_[len] / 255.0;
    *g_rate = G_[len] / 255.0;
    *b_rate = B_[len] / 255.0;
  }

 private:
  ColorBar() {
    Init();
    size_ = R_.size();
  }
  DECLARE_SINGLETON(ColorBar)

  int size_;
  std::vector<float> R_, G_, B_;
  void Init() {
    for (int i = 0; i < 255 - 140; i++) {
      R_.push_back(140 + i);
      G_.push_back(0.0);
      B_.push_back(0.0);
    }
    for (int i = 0; i < 255; i++) {
      R_.push_back(255);
      G_.push_back(i);
      B_.push_back(0.0);
    }
    for (int i = 0; i < 255; i++) {
      R_.push_back(255 - i);
      G_.push_back(255);
      B_.push_back(i);
    }
    for (int i = 0; i < 255; i++) {
      R_.push_back(0.0);
      G_.push_back(255 - i);
      B_.push_back(255);
    }
    for (int i = 0; i < 255 - 140; i++) {
      R_.push_back(0.0);
      G_.push_back(0.0);
      B_.push_back(255 - i);
    }
  }
};

}  // namespace pnc
}  // namespace xju
