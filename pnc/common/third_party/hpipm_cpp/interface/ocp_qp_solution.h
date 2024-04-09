/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_IPM_SOLUTION_H_
#define COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_IPM_SOLUTION_H_

#include "Eigen/Core"

namespace xju {
namespace pnc {
namespace hpipm {

/// @class OcpQpSolution
/// @brief Solution of the OCP-QP problem.
struct OcpQpSolution {
  /// @brief State. 
  Eigen::VectorXd x;

  /// @brief Control input. 
  Eigen::VectorXd u;

  /// @brief Costate (the Lagrange multiplier w.r.t the state equation). 
  Eigen::VectorXd pi;

  /// @brief Riccati matrix P. 
  Eigen::MatrixXd P;

  /// @brief Riccati vector s. 
  Eigen::VectorXd p;

  /// @brief Feedback gain. 
  Eigen::MatrixXd K;

  /// @brief Feedforward term. 
  Eigen::VectorXd k;
};

} // namespace hpipm
} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_IPM_SOLUTION_H_