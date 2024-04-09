/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#ifndef COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_IPM_SOLVER_SETTINGS_H_
#define COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_IPM_SOLVER_SETTINGS_H_

namespace xju {
namespace pnc {
namespace hpipm {

/// @class HpipmNode
/// @brief Solver mode
enum class HpipmMode {
  SpeedAbs,
  Speed,
  Balance,
  Robust
};

/// @class OCpQpIpmSolverSettings
/// @brief Ipm solver settings
struct OcpQpIpmSolverSettings {
  
  /// @brief Solver mode. Default is HpipmMode::Speed.
  HpipmMode mode = HpipmMode::Speed;
  
  /// @brief Maximum number of iterations. Must be positive. Default is 15.
  int iter_max = 14;

  /// @brief Minimum step size. Must be positive and less than 1.0. Default is 1.0e-08.
  double alpha_min = 1.0e-08;

  /// @brief Initial barrier parameter. Must be positive. Default is 1.0e+02.
  double mu0 = 1.0e+02;

  /// @brief Convergence criteria. Must be positive. Default is 1.0e-08.
  double tol_stat = 1.0e-08;

  /// @brief Convergence criteria. Must be positive. Default is 1.0e-08.
  double tol_eq = 1.0e-08;
  
  /// @brief Convergence criteria. Must be positive. Default is 1.0e-08.
  double tol_ineq = 1.0e-08;

  /// @brief Convergence criteria. Must be positive. Default is 1.0e-08.
  double tol_comp = 1.0e-08;

  /// @brief Regularization term. Must be positive. Default is 1.0e-12.
  double reg_prim = 1.0e-12;

  /// @brief Warm start flag (0: disable, 1: enable). Default is 0.
  int warm_start = 0;

  /// @brief Prediction-correction flag (0: disable, 1: enable). Default is 1.
  int pred_corr = 1;

  /// @brief Square-root Riccati flag (0: disable, 1: enable). Default is 1.
  int ric_alg = 1;

  /// @brief Use different step for primal and dual variables (0: disable, 1: enable). Default is 1.
  int split_step = 0;

  /// @brief Check the settings. If something is wrong, throws an exception.
  void CheckSettings() const;
};

} // namespace hpipm
} // namespace pnc
} // namespace xju

#endif // COMMON_THIRD_PARTY_HPIPM_CPP_OCP_QP_IPM_SOLVER_SETTINGS_H_