/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/speed_optimizer/piecewise_jerk_speed_optimizer.h"

#include "common/logger/logger.h"

namespace xju {
namespace planning {

namespace {
constexpr double kMaxVariableRange = 1.0e10;
}  // namespace

PiecewiseJerkSpeedOptimizer::PiecewiseJerkSpeedOptimizer(
    const size_t num_of_knots, const double delta_t,
    const std::array<double, 3>& s_init)
    : pnc::QPSolver(5000) {
  ACHECK(num_of_knots >= 2);
  num_of_knots_ = num_of_knots;
  kernel_dim_ = 5 * num_of_knots - 1;
  delta_t_ = delta_t;
  s_init_ = s_init;
  s_bounds_.resize(num_of_knots,
                   std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  ds_bounds_.resize(num_of_knots,
                    std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  dds_bounds_.resize(num_of_knots,
                     std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  ddds_bounds_.resize(num_of_knots - 1,
                      std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  slack_dds_bounds_.resize(
      num_of_knots, std::make_pair(-kMaxVariableRange, kMaxVariableRange));
  slack_ddds_bounds_.resize(
      num_of_knots - 1, std::make_pair(-kMaxVariableRange, kMaxVariableRange));
}

bool PiecewiseJerkSpeedOptimizer::Optimize(
    std::vector<double>* s,
    std::vector<double>* ds,
    std::vector<double>* dds) {
  if (!pnc::QPSolver::Optimize()) {
    return false;
  }
  ACHECK(s && ds && dds);
  s->clear();
  ds->clear();
  dds->clear();
  for (int i = 0; i < num_of_knots_; i++) {
    s->emplace_back(solution_[i] / scale_factor_[0]);
    ds->emplace_back(solution_[num_of_knots_ + i] / scale_factor_[1]);
    dds->emplace_back(solution_[2 * num_of_knots_ + i] / scale_factor_[2]);
  }
  return true;
}

void PiecewiseJerkSpeedOptimizer::set_s_bounds(
    const std::vector<std::pair<double, double>>& s_bounds) {
  ACHECK(s_bounds.size() == num_of_knots_);
  s_bounds_ = s_bounds;
}

void PiecewiseJerkSpeedOptimizer::set_ds_bounds(
    const std::vector<std::pair<double, double>>& ds_bounds) {
  ACHECK(ds_bounds.size() == num_of_knots_);
  ds_bounds_ = ds_bounds;
}

void PiecewiseJerkSpeedOptimizer::set_dds_bounds(
  const double dds_lower_bound, const double dds_upper_bound) {
  for (auto& it : dds_bounds_) {
    it.first = dds_lower_bound;
    it.second = dds_upper_bound;
  }
}

void PiecewiseJerkSpeedOptimizer::set_ddds_bounds(
    const double ddds_lower_bound, const double ddds_upper_bound) {
  for (auto& it : ddds_bounds_) {
    it.first = ddds_lower_bound;
    it.second = ddds_upper_bound;
  }
}

void PiecewiseJerkSpeedOptimizer::set_dds_slack_bounds(
    const double slack_lower_bound, const double slack_upper_bound) {
  for (auto& it : slack_dds_bounds_) {
    it.first = slack_lower_bound;
    it.second = slack_upper_bound;
  }
}

void PiecewiseJerkSpeedOptimizer::set_ddds_slack_bounds(
    const double slack_lower_bound, const double slack_upper_bound) {
  for (auto& it : slack_ddds_bounds_) {
    it.first = slack_lower_bound;
    it.second = slack_upper_bound;
  }
}

void PiecewiseJerkSpeedOptimizer::set_s_ref(
    const std::vector<double>& s_ref) {
  ACHECK(s_ref.size() == num_of_knots_);
  s_ref_ = s_ref;
}

void PiecewiseJerkSpeedOptimizer::set_ds_ref(
    const std::vector<double>& ds_ref) {
  ACHECK(ds_ref.size() == num_of_knots_);
  ds_ref_ = ds_ref;
}

void PiecewiseJerkSpeedOptimizer::set_kappa_ref(
    const std::vector<double>& kappa_ref) {
  ACHECK(kappa_ref.size() == num_of_knots_);
  kappa_ref_ = kappa_ref;
}

void PiecewiseJerkSpeedOptimizer::set_weight(
    const SpeedOptimizerWeights& config) {
  weight_dds_ = config.dds();
  weight_ddds_ = config.ddds();
  weight_a_c_ = config.a_c();
  weight_s_ref_ = config.s_ref();
  weight_ds_ref_ = config.ds_ref();
  weight_dds_slack_ = config.dds_slack();
  weight_ddds_slack_ = config.ddds_slack();
}

void PiecewiseJerkSpeedOptimizer::CalculateKernelAndOffset(
    std::vector<c_float>* P_data,
    std::vector<c_int>* P_indices,
    std::vector<c_int>* P_indptr,
    std::vector<c_float>* q) {
  ACHECK(P_data && P_indices && P_indptr && q);
  const int n = static_cast<int>(num_of_knots_);
  const int num_of_variables = 5 * n - 1;
  const int num_of_nonzeros = 5 * n - 1 + (n - 1);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(num_of_variables);
  int value_index = 0;

  // s(i)^2 * weight_s_ref_
  for (int i = 0; i < n; i++) {
    columns[i].emplace_back(
        i, weight_s_ref_ / (scale_factor_[0] * scale_factor_[0]));
    value_index++;
  }

  // s(i)'^2 * (weight_a_c_ * kappa_ref_(i) + weight_ds_ref_)
  for (int i = 0; i < n; i++) {
    columns[n + i].emplace_back(
        n + i, (weight_a_c_ * std::fabs(kappa_ref_[i]) + weight_ds_ref_) /
                   (scale_factor_[1] * scale_factor_[1]));
    value_index++;
  }

  const double delta_t_square = delta_t_ * delta_t_;

  // s(0)''^2 * (weight_dds_ + weight_ddds_ / (delta_t)^2)
  columns[2 * n].emplace_back(2 * n,
                              (weight_dds_ + weight_ddds_ / delta_t_square) /
                                  (scale_factor_[2] * scale_factor_[2]));
  value_index++;

  // s(i)'' * s(i+1)'' * (-2) * weight_ddds_ /  (delta_t)^2)
  for (int i = 1; i < n; i++) {
    columns[2 * n + i].emplace_back(2 * n + i - 1,
                                    (-1.0 * weight_ddds_ / delta_t_square) /
                                        (scale_factor_[2] * scale_factor_[2]));
    value_index++;
  }

  // s(i)''^2 * (weight_dds_ + 2 * weight_ddds_ / (delta_t)^2)
  for (int i = 1; i < n - 1; i++) {
    columns[2 * n + i].emplace_back(
        2 * n + i, (weight_dds_ + 2.0 * weight_ddds_ / delta_t_square) /
                       (scale_factor_[2] * scale_factor_[2]));
    value_index++;
  }

  // s(n-1)''^2 * (weight_dds_ + weight_ddds_ / (delta_t)^2)
  columns[3 * n - 1].emplace_back(
      3 * n - 1, (weight_dds_ + weight_ddds_ / delta_t_square) /
                     (scale_factor_[2] * scale_factor_[2]));
  value_index++;

  // dds_slack ^ 2 * weight_dds_slack_
  for (int i = 0; i < n; i++) {
    columns[3 * n + i].emplace_back(
        3 * n + i, weight_dds_slack_ / (scale_factor_[2] * scale_factor_[2]));
    value_index++;
  }

  // ddds_slack ^ 2 * weight_ddds_slack_
  for (int i = 0; i < n - 1; i++) {
    columns[4 * n + i].emplace_back(
        4 * n + i, weight_ddds_slack_ / (scale_factor_[2] * scale_factor_[2]));
    value_index++;
  }

  ACHECK(value_index == num_of_nonzeros);
  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    P_indptr->emplace_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->emplace_back(row_data_pair.second * 2.0);
      P_indices->emplace_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->emplace_back(ind_p);

  q->resize(num_of_variables, 0.0);
  for (int i = 0; i < n; i++) {
    q->at(i) = -2.0 * weight_s_ref_ * s_ref_[i] / scale_factor_[0];  // s(i)
    q->at(n + i) =
        -2.0 * weight_ds_ref_ * ds_ref_[i] / scale_factor_[1];  // s(i)'
  }
}

void PiecewiseJerkSpeedOptimizer::CalculateAffineConstraint(
    std::vector<c_float>* A_data,
    std::vector<c_int>* A_indices,
    std::vector<c_int>* A_indptr,
    std::vector<c_float>* lower_bounds,
    std::vector<c_float>* upper_bounds) {
  ACHECK(A_data && A_indices && A_indptr && lower_bounds && upper_bounds);
  const int num_of_variables =
      5 * num_of_knots_ - 1;  // s s' s'' dds_slack ddds_slack
  const int num_of_constraints = num_of_variables + 3 * (num_of_knots_ - 1) + 3;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);

  std::vector<std::vector<std::pair<c_int, c_float>>> variables(
      num_of_variables);
  int constraint_index = 0;

  CalculateAffineInequalityConstraint(
      lower_bounds, upper_bounds, &variables, &constraint_index);

  ACHECK(constraint_index == num_of_variables + num_of_knots_ - 1);

  CalculateAffineEquationConstraint(
      lower_bounds, upper_bounds, &variables, &constraint_index);

  CalculateAffineStartConstraint(
    lower_bounds, upper_bounds, &variables, &constraint_index);

  ACHECK(constraint_index == num_of_constraints);

  int ind_p = 0;
  for (int i = 0; i < num_of_variables; ++i) {
    A_indptr->emplace_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      A_data->emplace_back(variable_nz.second);
      A_indices->emplace_back(variable_nz.first);
      ++ind_p;
    }
  }
  A_indptr->emplace_back(ind_p);
}

void PiecewiseJerkSpeedOptimizer::CalculateAffineInequalityConstraint(
    std::vector<c_float>* const lower_bounds,
    std::vector<c_float>* const upper_bounds,
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables,
    int* const constraint_index) {
  ACHECK(lower_bounds && upper_bounds && variables && constraint_index);
  const int n = num_of_knots_;
  for (int i = 0; i < variables->size(); i++) {
    if (i < n) {
      // s
      (*variables)[i].emplace_back(*constraint_index, 1.0);
      lower_bounds->at(*constraint_index) =
          s_bounds_[i].first * scale_factor_[0];
      upper_bounds->at(*constraint_index) =
          s_bounds_[i].second * scale_factor_[0];
    } else if (i < 2 * n) {
      // s' 
      (*variables)[i].emplace_back(*constraint_index, 1.0);
      lower_bounds->at(*constraint_index) =
          ds_bounds_[i - n].first * scale_factor_[1];
      upper_bounds->at(*constraint_index) =
          ds_bounds_[i - n].second * scale_factor_[1];
    } else if (i < 3 * n) {
      // s'' - dds_slack
      (*variables)[i].emplace_back(*constraint_index, 1.0);
      (*variables)[n + i].emplace_back(*constraint_index, -1.0);
      lower_bounds->at(*constraint_index) =
          dds_bounds_[i - 2 * n].first * scale_factor_[2];
      upper_bounds->at(*constraint_index) =
          dds_bounds_[i - 2 * n].second * scale_factor_[2];
    } else if (i < 4 * n) {
      // dds_slack
      (*variables)[i].emplace_back(*constraint_index, 1.0);
      lower_bounds->at(*constraint_index) =
          slack_dds_bounds_[i - 3 * n].first * scale_factor_[2];
      upper_bounds->at(*constraint_index) =
          slack_dds_bounds_[i - 3 * n].second * scale_factor_[2];
    } else {
      // ddds_slack
      (*variables)[i].emplace_back(*constraint_index, 1.0);
      lower_bounds->at(*constraint_index) =
          slack_ddds_bounds_[i - 4 * n].first * scale_factor_[2];
      upper_bounds->at(*constraint_index) =
          slack_ddds_bounds_[i - 4 * n].second * scale_factor_[2];
    }
    (*constraint_index)++;
  }

  // s(i->i+1)''' -ddds_atsck
  // (s(i+1)'' - s(i)'') / delta_t - ddds_slack
  for (int i = 0; i < n - 1; i++) {
    (*variables)[2 * n + i].emplace_back(*constraint_index, -1.0);
    (*variables)[2 * n + i + 1].emplace_back(*constraint_index, 1.0);
    (*variables)[4 * n + i].emplace_back(*constraint_index, -1.0 * delta_t_);
    lower_bounds->at(*constraint_index) =
        ddds_bounds_[i].first * delta_t_ * scale_factor_[2];
    upper_bounds->at(*constraint_index) =
        ddds_bounds_[i].second * delta_t_ * scale_factor_[2];
    (*constraint_index)++;
  }
}

void PiecewiseJerkSpeedOptimizer::CalculateAffineEquationConstraint(
    std::vector<c_float>* const lower_bounds,
    std::vector<c_float>* const upper_bounds,
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables,
    int* const constraint_index) {
  ACHECK(lower_bounds && upper_bounds && variables && constraint_index);
  const int n = num_of_knots_;
  // s(i+1)' - s(i)' - 0.5 * delta_t * s(i)'' - 0.5 * delta_t * s(i+1)''
  for (int i = 0; i < n - 1; i++) {
    (*variables)[n + i].emplace_back(*constraint_index,
                                     -1.0 * scale_factor_[2]);
    (*variables)[n + i + 1].emplace_back(*constraint_index,
                                         1.0 * scale_factor_[2]);
    (*variables)[2 * n + i].emplace_back(*constraint_index,
                                         -0.5 * delta_t_ * scale_factor_[1]);
    (*variables)[2 * n + i + 1].emplace_back(
        *constraint_index, -0.5 * delta_t_ * scale_factor_[1]);
    lower_bounds->at(*constraint_index) = 0.0;
    upper_bounds->at(*constraint_index) = 0.0;
    (*constraint_index)++;
  }

  // s(i+1) - s(i) - s(i)' * delta_t - 1/3 * delta_t^2 * s(i)'' - 1/6 *
  // delta_t^2 * s(i+1)''
  const double delta_t_sqare = delta_t_ * delta_t_;
  for (int i = 0; i < n - 1; i++) {
    (*variables)[i].emplace_back(*constraint_index,
                                 -1.0 * scale_factor_[1] * scale_factor_[2]);
    (*variables)[i + 1].emplace_back(*constraint_index,
                                     1.0 * scale_factor_[1] * scale_factor_[2]);
    (*variables)[n + i].emplace_back(
        *constraint_index, -delta_t_ * scale_factor_[0] * scale_factor_[2]);
    (*variables)[2 * n + i].emplace_back(
        *constraint_index,
        -1.0 / 3.0 * delta_t_sqare * scale_factor_[0] * scale_factor_[1]);
    (*variables)[2 * n + i + 1].emplace_back(
        *constraint_index,
        -1.0 / 6.0 * delta_t_sqare * scale_factor_[0] * scale_factor_[1]);
    lower_bounds->at(*constraint_index) = 0.0;
    upper_bounds->at(*constraint_index) = 0.0;
    (*constraint_index)++;
  }
}

void PiecewiseJerkSpeedOptimizer::CalculateAffineStartConstraint(
    std::vector<c_float>* const lower_bounds,
    std::vector<c_float>* const upper_bounds,
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables,
    int* const constraint_index) {
  ACHECK(lower_bounds && upper_bounds && variables && constraint_index);
  const int n = num_of_knots_;
  // s_init
  (*variables)[0].emplace_back(*constraint_index, 1.0);
  lower_bounds->at(*constraint_index) = s_init_[0] * scale_factor_[0];
  upper_bounds->at(*constraint_index) = s_init_[0] * scale_factor_[0];
  (*constraint_index)++;

  (*variables)[n].emplace_back(*constraint_index, 1.0);
  lower_bounds->at(*constraint_index) = s_init_[1] * scale_factor_[1];
  upper_bounds->at(*constraint_index) = s_init_[1] * scale_factor_[1];
  (*constraint_index)++;

  (*variables)[2 * n].emplace_back(*constraint_index, 1.0);
  lower_bounds->at(*constraint_index) = s_init_[2] * scale_factor_[2];
  upper_bounds->at(*constraint_index) = s_init_[2] * scale_factor_[2];
  (*constraint_index)++;
}

void PiecewiseJerkSpeedOptimizer::SetWarmStartX() {
  start_x_.resize(kernel_dim_, 0.0);
  for (int i = 0; i < kernel_dim_ && i < warm_start_val_.size(); i++) {
    start_x_.emplace_back(static_cast<c_float>(warm_start_val_[i]));
  }
}
}  // namespace planning
}  // namespace xju
