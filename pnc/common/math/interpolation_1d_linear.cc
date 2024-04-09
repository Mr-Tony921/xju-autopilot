/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/interpolation_1d_linear.h"

#include <cmath>

#include "common/logger/logger.h"

namespace xju {
namespace pnc {
const double kDoubleEpsilon = 1e-6;

bool Interpolation1DLinear::Init(const DataType &xy) {
  if (xy.empty()) {
    AERROR << "empty input.";
    return false;
  }
  auto data(xy);
  std::sort(data.begin(), data.end());
  Eigen::VectorXd x(data.size());
  Eigen::VectorXd y(data.size());
  for (unsigned i = 0; i < data.size(); ++i) {
    x(i) = data[i].first;
    y(i) = data[i].second;
  }
  x_min_ = data.front().first;
  x_max_ = data.back().first;
  y_start_ = data.front().second;
  y_end_ = data.back().second;

  spline_.reset(new Eigen::Spline<double, 1>(
      Eigen::SplineFitting<Eigen::Spline<double, 1>>::Interpolate(
          y.transpose(),
          static_cast<Eigen::DenseIndex>(std::min<size_t>(x.size() - 1, 1)),
          ScaledValues(x))));
  return true;
}

double Interpolation1DLinear::Interpolate(double x) const {
  if (x < x_min_) {
    return y_start_;
  }
  if (x > x_max_) {
    return y_end_;
  }
  return (*spline_)(ScaledValue(x))(0);
}

double Interpolation1DLinear::ScaledValue(double x) const {
  if (std::fabs(x_max_ - x_min_) < kDoubleEpsilon) {
    return x_min_;
  }
  return (x - x_min_) / (x_max_ - x_min_);
}

Eigen::RowVectorXd Interpolation1DLinear::ScaledValues(
    Eigen::VectorXd const &x_vec) const {
  return x_vec.unaryExpr([this](double x) { return ScaledValue(x); })
      .transpose();
}

}  // namespace pnc
}  // namespace xju