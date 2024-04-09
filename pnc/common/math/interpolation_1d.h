/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "eigen3/unsupported/Eigen/Splines"  // poistion should before Eigen/Core
#include "Eigen/Core"

namespace xju {
namespace pnc {

class Interpolation1D {
 public:
  typedef std::vector<std::pair<double, double>> DataType;

  Interpolation1D() = default;

  bool Init(const DataType& xy);

  double Interpolate(double x) const;

 private:
  double ScaledValue(double x) const;

  Eigen::RowVectorXd ScaledValues(Eigen::VectorXd const& x_vec) const;

  double x_min_ = 0.0;
  double x_max_ = 0.0;
  double y_start_ = 0.0;
  double y_end_ = 0.0;

  std::unique_ptr<Eigen::Spline<double, 1>> spline_;
};

}  // namespace pnc
}  // namespace xju
