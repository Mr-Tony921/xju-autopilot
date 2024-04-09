/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_H_
#define COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_H_

#include <vector>

#include "Eigen/Core"

namespace xju {
namespace pnc {
namespace hpipm {

/// @class OcpQp
/// @brief The OCP-QP data class
struct OcpQp {
  /// @brief Dynamics matrix in x[k+1] = A[k] x[k] + B[k] u[k] + b[k].
  Eigen::MatrixXd A;

  /// @brief Dynamics matrix in x[k+1] = A[k] x[k] + B[k] u[k] + b[k].
  Eigen::MatrixXd B;

  /// @brief Dynamics matrix in x[k+1] = A[k] x[k] + B[k] u[k] + b[k].
  Eigen::MatrixXd b;

  /// @brief Cost vector in (1/2) * x[k]^T Q[k] x[k] + u[k]^T S[k] x[k] +
  ///                       (1/2) * u[k]^T R[k] u[k] + 
  ///                       q[k]^T x[k] + r[k]^T u[k]
  Eigen::MatrixXd Q;

  /// @brief Cost vector in (1/2) * x[k]^T Q[k] x[k] + u[k]^T S[k] x[k] +
  ///                       (1/2) * u[k]^T R[k] u[k] + 
  ///                       q[k]^T x[k] + r[k]^T u[k]
  Eigen::MatrixXd S;

  /// @brief Cost vector in (1/2) * x[k]^T Q[k] x[k] + u[k]^T S[k] x[k] +
  ///                       (1/2) * u[k]^T R[k] u[k] + 
  ///                       q[k]^T x[k] + r[k]^T u[k]
  Eigen::MatrixXd R;

  /// @brief Cost vector in (1/2) * x[k]^T Q[k] x[k] + u[k]^T S[k] x[k] +
  ///                       (1/2) * u[k]^T R[k] u[k] + 
  ///                       q[k]^T x[k] + r[k]^T u[k]
  Eigen::VectorXd q;

  /// @brief Cost vector in (1/2) * x[k]^T Q[k] x[k] + u[k]^T S[k] x[k] +
  ///                       (1/2) * u[k]^T R[k] u[k] + 
  ///                       q[k]^T x[k] + r[k]^T u[k]
  Eigen::VectorXd r;

  /// @brief Indices of box constrainted elements of x.
  std::vector<int> idxbx;

  /// @brief Lower bounds of box constraints on x.
  Eigen::VectorXd lbx;

  /// @brief Upper bounds of box constraints on x.
  Eigen::VectorXd ubx;

  /// @brief Masks on the lower bounds of box constraints on x.
  /// Each element must be composed only by 0 or 1.0
  Eigen::VectorXd lbx_mask;

  // @brief Masks on the upper bounds of box constraints on x.
  /// Each element must be composed only by 0 or 1.0
  Eigen::VectorXd ubx_mask;

  /// @brief Indices of box constrainted elements of u.
  std::vector<int> idxbu;

  /// @brief Lower bounds of box constraints on u.
  Eigen::VectorXd lbu;

  /// @brief Upper bounds of box constraints on u.
  Eigen::VectorXd ubu;

  /// @brief Masks on the lower bounds of box constraints on u.
  /// Each element must be composed only by 0 or 1.0
  Eigen::VectorXd lbu_mask;

  /// @brief Masks on the upper bounds of box constraints on u.
  /// Each element must be composed only by 0 or 1.0
  Eigen::VectorXd ubu_mask;

  /// @brief Constraint matrix in lg < C[k] x[k] + D[k] u[k] < ug
  Eigen::MatrixXd C;

  /// @brief Constraint matrix in lg < C[k] x[k] + D[k] u[k] < ug
  Eigen::MatrixXd D;

  /// @brief Constraint vector in lg < C[k] x[k] + D[k] u[k] < ug
  Eigen::VectorXd lg;

  /// @brief Constraint vector in lg < C[k] x[k] + D[k] u[k] < ug
  Eigen::VectorXd ug;

  /// @brief Masks on the lower bounds in lg < C[k] x[k] + D[k] u[k] < ug
  /// Each element must be composed only by 0 or 1.0
  Eigen::VectorXd lg_mask;

  /// @brief Masks on the upper bounds in lg < C[k] x[k] + D[k] u[k] < ug
  /// Each element must be composed only by 0 or 1.0
  Eigen::VectorXd ug_mask;

  /// @brief Matrix in the slack penalty (1/2) sl^T Zl sl + zl^T sl +
  ///                                    (1/2) su^T Zu su + zu^T su
  Eigen::MatrixXd Zl;

  /// @brief Matrix in the slack penalty (1/2) sl^T Zl sl + zl^T sl +
  ///                                    (1/2) su^T Zu su + zu^T su
  Eigen::MatrixXd Zu;

  /// @brief Matrix in the slack penalty (1/2) sl^T Zl sl + zl^T sl +
  ///                                    (1/2) su^T Zu su + zu^T su
  Eigen::VectorXd zl;

  /// @brief Matrix in the slack penalty (1/2) sl^T Zl sl + zl^T sl +
  ///                                    (1/2) su^T Zu su + zu^T su
  Eigen::VectorXd zu;

  /// @brief Indices of box constrainted elements of slack variables.
  std::vector<int> idxs;

  /// @brief Lower bounds of box constraints of slack variables.
  Eigen::VectorXd lls;

  /// @brief Upper bounds of box constraints of slack variables.
  Eigen::VectorXd lus;
};

} // namespace hpipm
} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_H_