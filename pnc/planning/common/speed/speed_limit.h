/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <algorithm>
#include <fstream>
#include "common/logger/logger.h"

namespace xju {
namespace planning {

class SpeedLimit final {
 public:
  SpeedLimit() = default;
  ~SpeedLimit() = default;
 
  void AppendSpeedLimit(const double s, const double v) {
    if (!speed_limit_points_.empty()) {
      DCHECK_GE(s, speed_limit_points_.back().first);
    }
    speed_limit_points_.emplace_back(s, v);
  }

  const std::vector<std::pair<double, double>>& speed_limit_points() const {
    return speed_limit_points_;
  }

  double GetSpeedLimitByS(const double s) const {
    CHECK_GE(speed_limit_points_.size(), 2U);
    DCHECK_GE(s, speed_limit_points_.front().first);

    auto compare_s = [](const std::pair<double, double>& point, const double s) {
      return point.first < s;
    };

    auto it_lower = std::lower_bound(speed_limit_points_.begin(),
                                     speed_limit_points_.end(), s, compare_s);
    
    if (it_lower == speed_limit_points_.end()) {
      return (it_lower - 1)->second;
    }
    return it_lower->second;
  }

  void Clear() {
    speed_limit_points_.clear();
  }

  std::string DebugString() const {
    std::stringstream ss;
    ss <<"SpeedLimit:\n";
    for(const auto& it : speed_limit_points_){
      ss << "s: "<< it.first<<"  speed_limit: "<<it.second<<"\n";  
    }
    return ss.str();
  }

 private:
  // use a vector to represent speed limit
  // the first number is s, the second number is v
  // It means at distance s from the start point, the speed limit is v.
  std::vector<std::pair<double, double>> speed_limit_points_;
};

} // namespace planning
} // namespace xju
