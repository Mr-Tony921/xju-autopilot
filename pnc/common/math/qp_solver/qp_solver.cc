/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "common/math/qp_solver/qp_solver.h"

#include <fstream>

#include "common/logger/logger.h"
#include "common/time/time.h"

namespace xju {
namespace pnc {

QPSolver::QPSolver(const int max_iter) : max_iter_(max_iter) {}

bool QPSolver::Optimize() {
  OSQPData* data = FormulateProblem();
  OSQPSettings* settings = SolverDefaultSetting();
  settings->max_iter = max_iter_;

  OSQPWorkspace* osqp_work = nullptr;
  osqp_work = osqp_setup(data, settings);

  SetWarmStartX();
  WarmStart(osqp_work);

  osqp_solve(osqp_work);
  // ADEBUG << DebugString(osqp_work);
  auto status = osqp_work->info->status_val;

  if (status < 0 || (status != 1 && status != 2)) {
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    LogOsqpSolverStatus(status);
    obj_val_ = std::numeric_limits<double>::infinity();
    return false;
  } else if (osqp_work->solution == nullptr) {
    osqp_cleanup(osqp_work);
    FreeData(data);
    c_free(settings);
    obj_val_ = std::numeric_limits<double>::infinity();
    AERROR << "The solution from OSQP is nullptr";
    return false;
  }

  solution_.resize(kernel_dim_);
  for (size_t i = 0; i < kernel_dim_; ++i) {
    solution_[i] = osqp_work->solution->x[i];
  }

  obj_val_ = osqp_work->info->obj_val;
  iter_num_ = osqp_work->info->iter;

  // clean up
  osqp_cleanup(osqp_work);
  FreeData(data);
  c_free(settings);
  return true;
}

OSQPSettings* QPSolver::SolverDefaultSetting() {
  OSQPSettings* settings =
      reinterpret_cast<OSQPSettings*>(c_malloc(sizeof(OSQPSettings)));
  osqp_set_default_settings(settings);
  settings->polish = true;
  settings->verbose = true;
  settings->scaled_termination = true;
  // settings->eps_abs = 1.0e-2;
  // settings->eps_rel = 1.0e-2;
  // settings->eps_prim_inf = 1.0e-5;
  // settings->eps_dual_inf = 1.0e-5;
  return settings;
}

OSQPData* QPSolver::FormulateProblem() {
  // calculate kernel
  std::vector<c_float> P_data;
  std::vector<c_int> P_indices;
  std::vector<c_int> P_indptr;

  // calculate offset
  std::vector<c_float> q;
  CalculateKernelAndOffset(&P_data, &P_indices, &P_indptr, &q);

  // calculate affine constraints
  std::vector<c_float> A_data;
  std::vector<c_int> A_indices;
  std::vector<c_int> A_indptr;
  std::vector<c_float> lower_bounds;
  std::vector<c_float> upper_bounds;
  CalculateAffineConstraint(&A_data, &A_indices, &A_indptr, &lower_bounds,
                            &upper_bounds);

  OSQPData* data = reinterpret_cast<OSQPData*>(c_malloc(sizeof(OSQPData)));
  if (lower_bounds.size() != upper_bounds.size()) {
    AERROR << "lower_bounds.size() != upper_bounds.size()";
    return data;
  }
  size_t num_affine_constraint = lower_bounds.size();

  data->n = kernel_dim_;
  data->m = num_affine_constraint;
  data->P =
      csc_matrix(kernel_dim_, kernel_dim_, P_data.size(), CopyData(P_data),
                 CopyData(P_indices), CopyData(P_indptr));
  data->q = CopyData(q);
  data->A =
      csc_matrix(num_affine_constraint, kernel_dim_, A_data.size(),
                 CopyData(A_data), CopyData(A_indices), CopyData(A_indptr));
  data->l = CopyData(lower_bounds);
  data->u = CopyData(upper_bounds);
  return data;
}

void QPSolver::FreeData(OSQPData* const data) {
  delete[] data->q;
  delete[] data->l;
  delete[] data->u;

  delete[] data->P->i;
  delete[] data->P->p;
  delete[] data->P->x;

  delete[] data->A->i;
  delete[] data->A->p;
  delete[] data->A->x;
}

void QPSolver::WarmStart(OSQPWorkspace* const work) {
  if (start_x_.size() != kernel_dim_) return;
  c_float* star_x = CopyData(start_x_);
  osqp_warm_start_x(work, star_x);
}

std::vector<double> QPSolver::solution() { return solution_; }

void QPSolver::LogOsqpSolverStatus(const int status) {
  if (status == 1) {
    AERROR << "OSQP sloved.";
  } else if (status == 2) {
    AERROR << "OSQP solved inaccurate.";
  } else if (status == 3) {
    AERROR << "OSQP primal infeasible inaccurate.";
  } else if (status == 4) {
    AERROR << "OSQP dual infeasible inaccurate.";
  } else if (status == -2) {
    AERROR << "OSQP maximum iterations reached.";
  } else if (status == -3) {
    AERROR << "OSQP primal infeasible.";
  } else if (status == -4) {
    AERROR << "OSQP dual infeasible.";
  } else if (status == -5) {
    AERROR << "OSQP interrupted by user.";
  } else if (status == -6) {
    AERROR << "OSQP run time limit reached.";
  } else if (status == -7) {
    AERROR << "OSQP problem non convex.";
  } else if (status == -10) {
    AERROR << "OSQP unsolved.";
  } else {
    AERROR << "OSQP Unknown Status.";
  }
}

std::string QPSolver::DebugString(const OSQPWorkspace* work) {
  std::stringstream ss;
  OSQPData* data;
  OSQPSettings* settings;
  OSQPInfo* info;
  int nnz;  // Number of nonzeros in the problem

  data = work->data;
  settings = work->settings;
  info = work->info;

  if (!settings->verbose) {
    return "";
  }

  // Number of nonzeros
  nnz = data->P->p[data->P->n] + data->A->p[data->A->n];

  ss << "\n----------------------------------------------------------------\n";
  ss << "           OSQP " << OSQP_VERSION
     << "  -  Operator Splitting QP Solver\n"
        "              (c) Bartolomeo Stellato,  Goran Banjac\n"
        "        University of Oxford  -  Stanford University 2018\n";
  ss << "----------------------------------------------------------------\n";

  // Print variables and constraints
  ss << "problem:  ";
  ss << "variables n = " << (int)data->n << " constraints m = " << (int)data->m
     << "\n";
  ss << "          nnz(P) + nnz(A) = " << nnz << "\n";

  // Print Settings
  ss << "settings: ";
  ss << "linear system solver = " << LINSYS_SOLVER_NAME[settings->linsys_solver]
     << "\n";

  if (work->linsys_solver->nthreads != 1) {
    ss << (int)work->linsys_solver->nthreads << " threads\n";
  }

  ss << "          eps_abs = " << settings->eps_abs
     << ", eps_rel = " << settings->eps_rel << "\n";
  ss << "          eps_prim_inf = " << settings->eps_prim_inf
     << ", eps_dual_inf = " << settings->eps_dual_inf << "\n";
  ss << "          rho = " << settings->rho;
  if (settings->adaptive_rho) ss << " (adaptive)";
  ss << "\n";

  ss << "          sigma = " << settings->sigma
     << ", alpha = " << settings->alpha;
  ss << ", max_iter = " << (int)settings->max_iter << "\n";

  if (settings->check_termination)
    ss << "          check_termination: on (interval "
       << (int)settings->check_termination << ")\n";
  else
    ss << "          check_termination: off\n";

#ifdef PROFILING
  if (settings->time_limit)
    ss << "          time_limit: " << settings->time_limit << "\n";
#endif /* ifdef PROFILING */

  if (settings->scaling)
    ss << "          scaling: on, ";
  else
    ss << "          scaling: off, ";

  if (settings->scaled_termination)
    ss << "scaled_termination: on\n";
  else
    ss << "scaled_termination: off\n";

  if (settings->warm_start)
    ss << "          warm start: on, ";
  else
    ss << "          warm start: off";

  if (settings->polish)
    ss << "polish: on\n";
  else
    ss << "polish: off\n";

  // footer
  ss << "footer:   ";
  ss << "status:               " << info->status << "\n";
  if (settings->polish && (info->status_val == OSQP_SOLVED)) {
    if (info->status_polish == 1) {
      ss << "          solution polish:      successful\n";
    } else if (info->status_polish < 0) {
      ss << "          solution polish:      unsuccessful\n";
    }
  }
  ss << "          number of iterations: " << (int)info->iter << "\n";
  if ((info->status_val == OSQP_SOLVED) ||
      (info->status_val == OSQP_SOLVED_INACCURATE)) {
    ss << "          optimal objective:    " << info->obj_val << "\n";
  }

#ifdef PROFILING
  ss << "          run time:             " << info->run_time << "\n";
#endif /* ifdef PROFILING */

#if EMBEDDED != 1
  ss << "          optimal rho estimate: " << info->rho_estimate << "\n";
#endif /* if EMBEDDED != 1 */

  ss << "----------------------------------------------------------------\n";
  return ss.str();
}

}  // namespace pnc
}  // namespace xju