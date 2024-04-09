/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <cmath>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "Eigen/SVD"

#include "common/logger/logger.h"


namespace xju {
namespace pnc {

/**
 * @brief Computes the Moore-Penrose pseudo-inverse of a given square matrix,
 * rounding all eigenvalues with absolute value bounded by epsilon to zero.
 *
 * @param m The square matrix to be pseudo-inverted
 * @param epsilon A small positive real number (optional; default is 1.0e-6).
 *
 * @return Moore-Penrose pseudo-inverse of the given matrix.
 */
template <typename T, unsigned int N>
Eigen::Matrix<T, N, N> PseudoInverse(const Eigen::Matrix<T, N, N> &m,
                                     const double epsilon = 1.0e-6) {
  Eigen::JacobiSVD<Eigen::Matrix<T, N, N>> svd(
      m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  return static_cast<Eigen::Matrix<T, N, N>>(
      svd.matrixV() *
      (svd.singularValues().array().abs() > epsilon)
          .select(svd.singularValues().array().inverse(), 0)
          .matrix()
          .asDiagonal() *
      svd.matrixU().adjoint());
}

/**
 * @brief Computes the Moore-Penrose pseudo-inverse of a given matrix,
 * rounding all eigenvalues with absolute value bounded by epsilon to zero.
 *
 * @param m The matrix to be pseudo-inverted
 * @param epsilon A small positive real number (optional; default is 1.0e-6).
 *
 * @return Moore-Penrose pseudo-inverse of the given matrix.
 */
template <typename T, unsigned int M, unsigned int N>
Eigen::Matrix<T, N, M> PseudoInverse(const Eigen::Matrix<T, M, N> &m,
                                     const double epsilon = 1.0e-6) {
  Eigen::Matrix<T, M, M> t = m * m.transpose();
  return static_cast<Eigen::Matrix<T, N, M>>(m.transpose() *
                                             PseudoInverse<T, M>(t));
}

/**
* @brief Computes bilinear transformation of the continuous to discrete form
for state space representation
* This assumes equation format of
*
*           dot_x = Ax + Bu
*           y = Cx + Du
*
*
*
* @param m_a, m_b, m_c, m_d are the state space matrix control matrix
*
* @return true or false.

 */

template <typename T, unsigned int L, unsigned int N, unsigned int O>
bool ContinuousToDiscrete(const Eigen::Matrix<T, L, L> &m_a,
                          const Eigen::Matrix<T, L, N> &m_b,
                          const Eigen::Matrix<T, O, L> &m_c,
                          const Eigen::Matrix<T, O, N> &m_d, const double ts,
                          Eigen::Matrix<T, L, L> *ptr_a_d,
                          Eigen::Matrix<T, L, N> *ptr_b_d,
                          Eigen::Matrix<T, O, L> *ptr_c_d,
                          Eigen::Matrix<T, O, N> *ptr_d_d) {
  if (ts <= 0.0) {
    AERROR << "ContinuousToDiscrete : ts is less than or equal to zero";
    return false;
  }

  // Only matrix_a is mandatory to be non-zeros in matrix
  // conversion.
  if (m_a.rows() == 0) {
    AERROR << "ContinuousToDiscrete: matrix_a size 0 ";
    return false;
  }

  Eigen::Matrix<T, L, L> m_identity = Eigen::Matrix<T, L, L>::Identity();
  *ptr_a_d = PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) *
             (m_identity + ts * 0.5 * m_a);

  *ptr_b_d =
      std::sqrt(ts) * PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) * m_b;

  *ptr_c_d =
      std::sqrt(ts) * m_c * PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a);

  *ptr_d_d =
      0.5 * m_c * PseudoInverse<T, L>(m_identity - ts * 0.5 * m_a) * m_b + m_d;

  return true;
}

bool ContinuousToDiscrete(const Eigen::MatrixXd &m_a,
                          const Eigen::MatrixXd &m_b,
                          const Eigen::MatrixXd &m_c,
                          const Eigen::MatrixXd &m_d, const double ts,
                          Eigen::MatrixXd *ptr_a_d, Eigen::MatrixXd *ptr_b_d,
                          Eigen::MatrixXd *ptr_c_d, Eigen::MatrixXd *ptr_d_d) {
  if (ts <= 0.0) {
    AERROR << "ContinuousToDiscrete : ts is less than or equal to zero";
    return false;
  }

  // Only matrix_a is mandatory to be non-zeros in matrix
  // conversion.
  if (m_a.rows() == 0) {
    AERROR << "ContinuousToDiscrete: matrix_a size 0 ";
    return false;
  }

  if (m_a.cols() != m_b.rows() || m_b.cols() != m_d.cols() ||
      m_c.rows() != m_d.rows() || m_a.cols() != m_c.cols()) {
    AERROR << "ContinuousToDiscrete: matrix dimensions mismatch";
    return false;
  }

  Eigen::MatrixXd m_identity =
      Eigen::MatrixXd::Identity(m_a.cols(), m_a.rows());

  *ptr_a_d =
      (m_identity - ts * 0.5 * m_a).inverse() * (m_identity + ts * 0.5 * m_a);

  *ptr_b_d = std::sqrt(ts) * (m_identity - ts * 0.5 * m_a).inverse() * m_b;

  *ptr_c_d = std::sqrt(ts) * m_c * (m_identity - ts * 0.5 * m_a).inverse();

  *ptr_d_d = 0.5 * m_c * (m_identity - ts * 0.5 * m_a).inverse() * m_b + m_d;

  return true;
}

template <typename T, int M, int N, typename D>
void DenseToCSCMatrix(const Eigen::Matrix<T, M, N> &dense_matrix,
                      std::vector<T> *data, std::vector<D> *indices,
                      std::vector<D> *indptr) {
  static constexpr double epsilon = 1e-9;
  int data_count = 0;
  for (int c = 0; c < dense_matrix.cols(); ++c) {
    indptr->emplace_back(data_count);
    for (int r = 0; r < dense_matrix.rows(); ++r) {
      if (std::fabs(dense_matrix(r, c)) < epsilon) {
        continue;
      }
      data->emplace_back(dense_matrix(r, c));
      ++data_count;
      indices->emplace_back(r);
    }
  }
  indptr->emplace_back(data_count);
}

}  // namespace pnc
}  // namespace xju
