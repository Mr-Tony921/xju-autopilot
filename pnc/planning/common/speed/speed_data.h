/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>

#include "pnc_point.pb.h"

namespace xju {
namespace planning {

class SpeedData : public std::vector<pnc::SpeedPoint> {
 public:
  SpeedData() = default;
  ~SpeedData() = default;

  explicit SpeedData(std::vector<pnc::SpeedPoint> speed_points);

  void AppendSpeedPoint(const double t, const double s, const double v, 
                        const double a, const double da);
  
  bool EvaluateByTime(const double t, 
                      pnc::SpeedPoint* const speed_point) const;
  
  bool EvaluateByS(const double s,
                   pnc::SpeedPoint* const speed_point) const;

  double TotalTime() const;

  double TotalLength() const;
  
  std::string DebugString() const;
};

} // namespace planning
} // namespace xju
