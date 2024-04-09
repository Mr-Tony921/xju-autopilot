/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "common/math/vec2d.h"
#include "common/logger/logger.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>

namespace xju {
namespace pnc {

class CurveMath {
static constexpr int max_poly_order = 3;
 public:
  CurveMath() = delete;
  // x(t) y(t)
  /**
   * @brief Compute the curvature (kappa) given curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @return the curvature
   */
  static double ComputeCurvature(const double dx, const double d2x,
                                 const double dy, const double d2y);

  /**
   * @brief Compute the curvature change rate w.r.t. curve length (dkappa) given
   * curve X = (x(t), y(t))
   *        which t is an arbitrary parameter.
   * @param dx dx / dt
   * @param d2x d(dx) / dt
   * @param dy dy / dt
   * @param d2y d(dy) / dt
   * @param d3x d(d2x) / dt
   * @param d3y d(d2y) / dt
   * @return the curvature change rate
   */
  static double ComputeCurvatureDerivative(
      const double dx, const double d2x,
      const double d3x, const double dy,
      const double d2y, const double d3y);

  // y(x)
  static double ComputeCurvature(const double dy, const double d2y);

  static double ComputeCurvatureDerivative(
      const double dy, const double d2y, const double d3y);

  // Use discerete point calculate curvature
  static double ComputeCurvature(
      const Vec2d p1, const Vec2d p2, const Vec2d p3);

  // fit polynomial function with QR decomposition (using Eigen 3)
  static bool PolyFit(const std::vector<Eigen::Matrix<float, 2, 1>>& pos_vec,
                      const int& order,
                      Eigen::Matrix<float, max_poly_order + 1, 1>* coeff,
                      const bool& is_x_axis = true);

  // @brief: ransac fitting to estimate the coefficients of linear system
  static bool RansacFitting(
      const std::vector<Eigen::Matrix<float, 2, 1>>& pos_vec,
      std::vector<Eigen::Matrix<float, 2, 1>>* selected_points,
      Eigen::Matrix<float, 4, 1>* coeff,
      const int max_iters = 100,
      const int N = 5,
      const float inlier_thres = static_cast<float>(0.1));

 private:
  static double ComputeCurvatureDirection(
      const Vec2d p1, const Vec2d p2, const Vec2d p3);
};

}  // namespace pnc
}  // namespace xju
