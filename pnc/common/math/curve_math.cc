/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/curve_math.h"

#include <cmath>

namespace xju {
namespace pnc {

// x(t) y(t)
// kappa = (dx * d2y - dy * d2x) / [(dx * dx + dy * dy)^(3/2)]
double CurveMath::ComputeCurvature(
    const double dx, const double d2x,
    const double dy, const double d2y) {
  const double a = dx * d2y - dy * d2x;
  auto norm_square = dx * dx + dy * dy;
  auto norm = std::sqrt(norm_square);
  const double b = norm * norm_square + 1e-6;
  return a / b;
}

double CurveMath::ComputeCurvatureDerivative(
    const double dx, const double d2x,
    const double d3x, const double dy,
    const double d2y, const double d3y) {
  const double a = dx * d2y - dy * d2x;
  const double b = dx * d3y - dy * d3x;
  const double c = dx * d2x + dy * d2y;
  const double d = dx * dx + dy * dy;
  return (b * d - 3.0 * a * c) / (d * d * d);
}

// y(x)
double CurveMath::ComputeCurvature(const double dy, const double d2y) {
  double a = d2y;
  double norm_square = 1 + dy * dy;
  double norm = std::sqrt(norm_square);
  double b = norm_square * norm;
  return a / b;
}

double CurveMath::ComputeCurvatureDerivative(
    const double dy, const double d2y, const double d3y) {
  double a = d2y;
  double b = d3y;
  double c = dy * d2y;
  double d = 1.0 + dy * dy;
  return (b * d - 3.0 * a * c) / (d * d * d);
}

// https://mathworld.wolfram.com/SSSTheorem.html
double CurveMath::ComputeCurvature(
    const Vec2d p1, const Vec2d p2, const Vec2d p3) {
  double curvature = 0;
  double a, b, c, s, K;
  a = p1.DistanceTo(p2);
  b = p2.DistanceTo(p3);
  c = p1.DistanceTo(p3);
  s = (a + b + c) / 2;
  K = std::sqrt(std::fabs(s * (s - a) * (s - b) * (s - c)));
  if (a * b * c) {
    curvature = 4 * K / (a * b * c);
  } else {
    curvature = 0;
  }
  return curvature * ComputeCurvatureDirection(p1, p2, p3);
}

double CurveMath::ComputeCurvatureDirection(
    const Vec2d p1, const Vec2d p2, const Vec2d p3) {
  Vec2d vec12 = p2 - p1;
  vec12.SelfRotate(M_PI_2);
  Vec2d vec23 = p3 - p2;
  double val = vec12.InnerProd(vec23);
  if (val >= 0) {
    return 1;
  } else {
    return -1;
  }
}

bool CurveMath::PolyFit(
    const std::vector<Eigen::Matrix<float, 2, 1>>& pos_vec,
    const int& order,
    Eigen::Matrix<float, max_poly_order + 1, 1>* coeff,
    const bool& is_x_axis) {
  if (coeff == nullptr) {
    AERROR << "The coefficient pointer is NULL.";
    return false;
  }

  if (order > max_poly_order) {
    AERROR << "The order of polynomial must be smaller than " << max_poly_order;
    return false;
  }

  int n = static_cast<int>(pos_vec.size());
  if (n <= order) {
    AERROR << "The number of points should be larger than the order. #points = "
           << pos_vec.size();
    return false;
  }

  // create data matrix
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> A(n, order + 1);
  Eigen::Matrix<float, Eigen::Dynamic, 1> y(n);
  Eigen::Matrix<float, Eigen::Dynamic, 1> result;
  for (int i = 0; i < n; ++i) {
    for (int j = 0; j <= order; ++j) {
      A(i, j) = static_cast<float>(
          std::pow(is_x_axis ? pos_vec[i].x() : pos_vec[i].y(), j));
    }
    y(i) = is_x_axis ? pos_vec[i].y() : pos_vec[i].x();
  }

  // solve linear least squares
  result = A.householderQr().solve(y);
  assert(result.size() == order + 1);

  for (int j = 0; j <= max_poly_order; ++j) {
    (*coeff)(j) = (j <= order) ? result(j) : static_cast<float>(0);
  }

  return true;
}

bool CurveMath::RansacFitting(
    const std::vector<Eigen::Matrix<float, 2, 1>>& pos_vec,
    std::vector<Eigen::Matrix<float, 2, 1>>* selected_points,
    Eigen::Matrix<float, 4, 1>* coeff,
    const int max_iters,
    const int N,
    const float inlier_thres) {
  if (coeff == nullptr) {
    AERROR << "The coefficient pointer is NULL.";
    return false;
  }

  selected_points->clear();

  int n = static_cast<int>(pos_vec.size());
  int q1 = static_cast<int>(n / 4);
  int q2 = static_cast<int>(n / 2);
  int q3 = static_cast<int>(n * 3 / 4);
  if (n < N) {
    AERROR << "The number of points should be larger than the order. #points = "
           << pos_vec.size();
    return false;
  }

  std::vector<int> index(3, 0);
  int max_inliers = 0;
  float min_residual = std::numeric_limits<float>::max();
  float early_stop_ratio = 0.95f;
  float good_lane_ratio = 0.666f;
  for (int j = 0; j < max_iters; ++j) {
    index[0] = std::rand() % q2;
    index[1] = q2 + std::rand() % q1;
    index[2] = q3 + std::rand() % q1;

    Eigen::Matrix<float, 3, 3> matA;
    matA << pos_vec[index[0]](0) * pos_vec[index[0]](0), pos_vec[index[0]](0),
        1, pos_vec[index[1]](0) * pos_vec[index[1]](0), pos_vec[index[1]](0), 1,
        pos_vec[index[2]](0) * pos_vec[index[2]](0), pos_vec[index[2]](0), 1;

    Eigen::FullPivLU<Eigen::Matrix<float, 3, 3>> mat(matA);
    mat.setThreshold(1e-5f);
    if (mat.rank() < 3) {
      ADEBUG << "matA: " << matA;
      ADEBUG << "Matrix is not full rank (3). The rank is: " << mat.rank();
      continue;
    }

    // Since Eigen::solver was crashing, simple inverse of 3x3 matrix is used
    // Note that Eigen::inverse of 3x3 and 4x4 is a closed form solution
    Eigen::Matrix<float, 3, 1> matB;
    matB << pos_vec[index[0]](1), pos_vec[index[1]](1), pos_vec[index[2]](1);
    Eigen::Vector3f c =
        static_cast<Eigen::Matrix<float, 3, 1>>(matA.inverse() * matB);
    if (!(matA * c).isApprox(matB)) {
      ADEBUG << "No solution.";
      continue;
    }

    int num_inliers = 0;
    float residual = 0;
    float y = 0;
    for (int i = 0; i < n; ++i) {
      y = pos_vec[i](0) * pos_vec[i](0) * c(0) + pos_vec[i](0) * c(1) + c(2);
      if (std::abs(y - pos_vec[i](1)) <= inlier_thres)
        ++num_inliers;
      residual += std::abs(y - pos_vec[i](1));
    }

    if (num_inliers > max_inliers ||
        (num_inliers == max_inliers && residual < min_residual)) {
      (*coeff)(3) = 0;
      (*coeff)(2) = c(0);
      (*coeff)(1) = c(1);
      (*coeff)(0) = c(2);
      max_inliers = num_inliers;
      min_residual = residual;
    }

    if (max_inliers > early_stop_ratio * n)
      break;
  }

  if (static_cast<float>(max_inliers) / n < good_lane_ratio)
    return false;

  // std::vector<Eigen::Matrix<float, 2, 1>> tmp = *pos_vec;
  // pos_vec.clear();
  for (int i = 0; i < n; ++i) {
    float y = pos_vec[i](0) * pos_vec[i](0) * (*coeff)(2) +
              pos_vec[i](0) * (*coeff)(1) + (*coeff)(0);
    if (std::abs(y - pos_vec[i](1)) <= inlier_thres) {
      selected_points->push_back(pos_vec[i]);
    }
  }
  return true;
}
}  // namespace pnc
}  // namespace xju
