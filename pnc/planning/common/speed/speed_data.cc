/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/speed/speed_data.h"

#include <algorithm>
#include <mutex>
#include <fstream>
#include "common/util/point_factory.h"
#include "common/math/linear_interpolation.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

SpeedData::SpeedData(std::vector<pnc::SpeedPoint> speed_points)
    : std::vector<pnc::SpeedPoint>(std::move(speed_points)) {
  std::sort(begin(), end(), [](const pnc::SpeedPoint& p1, 
    const pnc::SpeedPoint& p2) {
    return p1.t() < p2.t();
  }); 
}

void SpeedData::AppendSpeedPoint(const double t, const double s, 
                                 const double v, const double a, 
                                 const double da) {
  if (!empty()) {
    ACHECK(back().t() < t);
  }
  push_back(pnc::PointFactory::ToSpeedPoint(s, t, v, a, da));
}

bool SpeedData::EvaluateByTime(
    const double t, pnc::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().t() < t + 1.0e-6 && t - 1.0e-6 < back().t())) {
    return false;
  }

  auto comp = [](const pnc::SpeedPoint& sp, const double t) {
    return sp.t() < t;
  };

  auto it_lower = std::lower_bound(begin(), end(), t, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double t0 = p0.t();
    double t1 = p1.t();

    speed_point->Clear();
    speed_point->set_s(pnc::lerp(p0.s(), t0, p1.s(), t1, t));
    speed_point->set_t(t);
    if (p0.has_v() && p1.has_v()) {
      speed_point->set_v(pnc::lerp(p0.v(), t0, p1.v(), t1, t));
    }
    if (p0.has_a() && p1.has_a()) {
      speed_point->set_a(pnc::lerp(p0.a(), t0, p1.a(), t1, t));
    }
    if (p0.has_da() && p1.has_da()) {
      speed_point->set_da(pnc::lerp(p0.da(), t0, p1.da(), t1, t));
    }
  }
  return true;

}

bool SpeedData::EvaluateByS(
    const double s, pnc::SpeedPoint* const speed_point) const {
  if (size() < 2) {
    return false;
  }
  if (!(front().s() < s + 1.0e-6 && s - 1.0e-6 < back().s())) {
    return false;
  }

  auto comp = [](const pnc::SpeedPoint& sp, const double s) {
    return sp.s() < s;
  };

  auto it_lower = std::lower_bound(begin(), end(), s, comp);
  if (it_lower == end()) {
    *speed_point = back();
  } else if (it_lower == begin()) {
    *speed_point = front();
  } else {
    const auto& p0 = *(it_lower - 1);
    const auto& p1 = *it_lower;
    double s0 = p0.s();
    double s1 = p1.s();

    speed_point->Clear();
    speed_point->set_s(s);
    speed_point->set_t(pnc::lerp(p0.t(), s0, p1.t(), s1, s));
    if (p0.has_v() && p1.has_v()) {
      speed_point->set_v(pnc::lerp(p0.v(), s0, p1.v(), s1, s));
    }
    if (p0.has_a() && p1.has_a()) {
      speed_point->set_a(pnc::lerp(p0.a(), s0, p1.a(), s1, s));
    }
    if (p0.has_da() && p1.has_da()) {
      speed_point->set_da(pnc::lerp(p0.da(), s0, p1.da(), s1, s));
    }
  }
  return true;              
}

double SpeedData::TotalTime() const {
  if (empty()) {
    return 0.0;
  }
  return back().t() - front().t();
}

double SpeedData::TotalLength() const {
  if (empty()) {
    return 0.0;
  }
  return back().s() - front().s();
}

std::string SpeedData::DebugString() const {
  std::stringstream ss;
  ss << "speed_data size: "<<size()<<"\n";
  int i = 0;
  for(auto iter = begin();iter != end();iter++){
    ss << "["<<i++<<"]  t: "<<iter->t()
       << "  s: "<<iter->s()
       << "  v: "<<iter->v()
       << "  a: "<<iter->a()
       << "  da: "<<iter->da() <<"\n";
  }
  return ss.str();
}

} // namespace planning
} // namespace xju
