/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/hpipm_cpp/hpipm_cpp.h"

#include <vector>

#include "Eigen/Core"
#include "gtest/gtest.h"
#include "common/logger/logger.h"

namespace xju {
namespace pnc {
namespace hpipm {

TEST(HpipmCpp, OcpQp) {
  int N = 5;
  const Eigen::MatrixXd A = (Eigen::MatrixXd(2, 2) << 1.0, 1.0,
                                                      0.0, 1.0).finished();
  const Eigen::MatrixXd B = (Eigen::MatrixXd(2, 1) << 0.0, 1.0).finished();
  const Eigen::VectorXd b = (Eigen::VectorXd(2) << 0.0, 0.0).finished();
  const Eigen::MatrixXd Q = (Eigen::MatrixXd(2, 2) << 1.0, 0.0, 
                                                      0.0, 1.0).finished();
  const Eigen::MatrixXd R = (Eigen::MatrixXd(1, 1) << 1.0).finished();
  const Eigen::MatrixXd S = (Eigen::MatrixXd(1, 2) << 0.0, 0.0).finished();
  const Eigen::MatrixXd q = (Eigen::VectorXd(2) << 1.0, 1.0).finished();
  const Eigen::VectorXd r = (Eigen::VectorXd(1) << 0.0).finished();
  const Eigen::VectorXd x0 = (Eigen::VectorXd(2) << 1.0, 1.0).finished();

  std::vector<OcpQp> qp(N + 1);
  for (int i = 0; i < N; ++i) {
    qp[i].A = A;
    qp[i].B = B;
    qp[i].b = b; 
  }
  // cost 
  for (int i = 0; i < N; ++i) {
    qp[i].Q = Q;
    qp[i].R = R;
    qp[i].S = S;
    qp[i].q = q;
    qp[i].r = r;
  }
  // no control variables
  qp[N].Q = Q;
  qp[N].q = q;

  OcpQpIpmSolverSettings solver_settings;
  solver_settings.mode = HpipmMode::Balance;
  solver_settings.iter_max = 30;
  solver_settings.alpha_min = 1e-8;
  solver_settings.mu0 = 1e2;
  solver_settings.tol_stat = 1e-04;
  solver_settings.tol_eq = 1e-04;
  solver_settings.tol_ineq = 1e-04;
  solver_settings.tol_comp = 1e-04;
  solver_settings.reg_prim = 1e-12;
  solver_settings.warm_start = 0;
  solver_settings.pred_corr = 1;
  solver_settings.ric_alg = 0;
  solver_settings.split_step = 1;

  std::vector<OcpQpSolution> solution(N + 1);
  OcpQpIpmSolver solver(qp, solver_settings);
  const auto res = solver.solve(x0, qp, solution);
  AINFO << "QP result: " << res;

  AINFO << "OCP QP primal solution: ";
  for (int i = 0; i <= N; ++i) {
    AINFO << "x[" << i << "]: " << solution[i].x.transpose();
  }
  for (int i = 0; i < N; ++i) {
    AINFO << "u[" << i << "]: " << solution[i].u.transpose();
  }

  AINFO << "OCP QP dual solution (Lagrange multipliers): ";
  for (int i = 0; i <= N; ++i) {
    AINFO << "pi[" << i << "]: " << solution[i].pi.transpose();
  }

  const auto& stat = solver.solver_statistics();
  AINFO << stat; 
};

} // namespace hpipm
} // namespace pnc
} // namespace xju