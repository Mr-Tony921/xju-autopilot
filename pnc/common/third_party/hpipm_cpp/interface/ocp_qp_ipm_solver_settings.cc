/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/third_party/hpipm_cpp/interface/ocp_qp_ipm_solver_settings.h"

#include "common/logger/logger.h"

namespace xju {
namespace pnc {
namespace hpipm {

void OcpQpIpmSolverSettings::CheckSettings() const {
  ACHECK(iter_max > 0) << "OcpQpIpmSolverSettings.iter_max must be positive";
  ACHECK(alpha_min > 0.0) << "OcpQpIpmSolverSettings.alpha_min must be positive";
  ACHECK(alpha_min <= 1.0) << "OcpQpIpmSolverSettings.alpha_min must be less than 1.0";
  ACHECK(mu0 > 0.0) << "OcpQpIpmSolverSettings.mu0 must be positive";
  ACHECK(tol_stat > 0.0) << "OcpQpIpmSolverSettings.tol_stat must be positive";
  ACHECK(tol_eq > 0.0) << "OcpQpIpmSolverSettings.tol_eq must be positive";
  ACHECK(tol_ineq > 0.0) << "OcpQpIpmSolverSettings.tol_ineq must be positive";
  ACHECK(tol_comp > 0.0) << "OcpQpIpmSolverSettings.tol_comp must be positive";
  ACHECK(reg_prim > 0.0) << "OcpQpIpmSolverSettings.reg_prim must be positive";
}

} // namespace hpipm
} // namespace pnc
} // namespace xju