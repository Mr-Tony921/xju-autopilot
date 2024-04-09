/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <array>
#include <vector>

#include "common/math/qp_solver/qp_solver.h"
#include "planning_task_config.pb.h"

namespace xju {
namespace planning {

class PiecewiseJerkSpeedOptimizer : public pnc::QPSolver {
 public:
  PiecewiseJerkSpeedOptimizer(const size_t num_of_knots, const double delta_t,
                              const std::array<double, 3>& s_init);
  ~PiecewiseJerkSpeedOptimizer() = default;

  bool Optimize(std::vector<double>* s, std::vector<double>* ds,
                std::vector<double>* dds);

  void set_s_bounds(const std::vector<std::pair<double, double>>& s_bounds);

  void set_ds_bounds(const std::vector<std::pair<double, double>>& ds_bounds);

  void set_dds_bounds(const double dds_lower_bound,
                      const double dds_upper_bound);

  void set_ddds_bounds(const double ddds_lower_bound,
                       const double ddds_upper_bounds);

  void set_dds_slack_bounds(const double slack_lower_bound,
                            const double slack_upper_bound);

  void set_ddds_slack_bounds(const double slack_lower_bound,
                             const double slack_upper_bound);

  void set_s_ref(const std::vector<double>& s_ref);

  void set_ds_ref(const std::vector<double>& ds_ref);

  void set_kappa_ref(const std::vector<double>& kappa_ref);

  void set_weight(const SpeedOptimizerWeights& config);

  // warm start
  void set_warm_start_val(const std::vector<double>& warm_start_val) {
    warm_start_val_ = warm_start_val;
  }

 protected:
  void CalculateKernelAndOffset(
      std::vector<c_float>* P_data,
      std::vector<c_int>* P_indices,
      std::vector<c_int>* P_indptr,
      std::vector<c_float>* q) override;

  void CalculateAffineConstraint(
      std::vector<c_float>* A_data,
      std::vector<c_int>* A_indices,
      std::vector<c_int>* A_indptr,
      std::vector<c_float>* lower_bounds,
      std::vector<c_float>* upper_bounds) override;

  void SetWarmStartX() override;

 private:
  void CalculateAffineInequalityConstraint(
      std::vector<c_float>* const lower_bounds,
      std::vector<c_float>* const upper_bounds,
      std::vector<std::vector<std::pair<c_int, c_float>>>* const variables,
      int* const constraint_index);

  void CalculateAffineEquationConstraint(
      std::vector<c_float>* const lower_bounds,
      std::vector<c_float>* const upper_bounds,
      std::vector<std::vector<std::pair<c_int, c_float>>>* const variables,
      int* const constraint_index);

  void CalculateAffineStartConstraint(
      std::vector<c_float>* const lower_bounds,
      std::vector<c_float>* const upper_bounds,
      std::vector<std::vector<std::pair<c_int, c_float>>>* const variables,
      int* const constraint_index);

 private:
  int num_of_knots_;
  double delta_t_;
  std::array<double, 3> s_init_;
  std::array<double, 3> scale_factor_ = {{1.0, 5.0, 6.5}};

  double weight_dds_;
  double weight_ddds_;
  double weight_a_c_;
  double weight_s_ref_;
  double weight_ds_ref_;
  double weight_dds_slack_;
  double weight_ddds_slack_;

  std::vector<double> warm_start_val_;  // 6*n-1   

  // ref
  std::vector<double> s_ref_;      // n
  std::vector<double> ds_ref_;     // n
  std::vector<double> kappa_ref_;  // n

  // bound
  std::vector<std::pair<double, double>> s_bounds_;           // n
  std::vector<std::pair<double, double>> ds_bounds_;          // n
  std::vector<std::pair<double, double>> dds_bounds_;         // n
  std::vector<std::pair<double, double>> ddds_bounds_;        // n-1
  std::vector<std::pair<double, double>> slack_dds_bounds_;   // n
  std::vector<std::pair<double, double>> slack_ddds_bounds_;  // n-1
};

}  // namespace planning
}  // namespace xju
