/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <array>
#include <vector>
#include <unordered_map>

#include "common/math/qp_solver/qp_solver.h"
#include "pnc_map/reference_line.h"
#include "planning/common/path/path_boundary.h"
#include "planning/common/path/path_data.h"
#include "task_config.pb.h"
#include "vehicle_config.pb.h"
#include "Eigen/Core"

namespace xju {
namespace planning {

struct SlState {
  double s;
  double l;
  double d_theta;
  double kappa;
  double dkappa;
};

class MassPointModelOptimizer : public pnc::QPSolver {
 public:
  MassPointModelOptimizer();
  ~MassPointModelOptimizer() = default;

  void Init();
  void set_weights(const PathOptimizerWeights& weights);
  void SetCarSpeedInfo(const double init_speed, const double target_speed);

  bool OptimizePath(
      const pnc_map::ReferenceLine& reference_line,
      const pnc::PathPoint& start_point,
      const PathBoundary* const boundary);

  bool GetOptimizePath(PathData* const path_data);
  double cost() const {
    return cost_;
  }

 protected:
  void CalculateKernelAndOffset(
      std::vector<c_float>* const P_data, 
      std::vector<c_int>* const P_indices, 
      std::vector<c_int>* const P_indptr, 
      std::vector<c_float>* const q) override;

  void CalculateAffineConstraint(
      std::vector<c_float>* const A_data, 
      std::vector<c_int>* const A_indices, 
      std::vector<c_int>* const A_indptr,
      std::vector<c_float>* const lower_bounds, 
      std::vector<c_float>* const upper_bounds) override;

//   OSQPSettings* SolverDefaultSetting();
  void SetWarmStartX() override;

 private:
  void InitLocalVirables(
      const pnc::PathPoint& start_point,
      const PathBoundary* const boundary);

  void CalculateStartPointConstraint(
      const pnc::PathPoint& start_point);

  void CalculateReferencePoints(
      const pnc_map::ReferenceLine& reference_line);

  void CalculateVehicleStateSpace(
      const double kappa, const double delta_s,
      Eigen::MatrixXd* const sys, Eigen::MatrixXd* const control, 
      Eigen::MatrixXd* const dis);

  void CalculateVehicleStateSpace(
      const int index,
      Eigen::MatrixXd* const sys, Eigen::MatrixXd* const control, 
      Eigen::MatrixXd* const dis);

  std::pair<double, double> CalculateBoundByS(
      const std::vector<std::pair<double, double>>& bounds,
      const double s);

  bool Check();

  void CalculateStateCostTerm(
      std::vector<std::vector<std::pair<c_int, c_float>>>* const columns, 
      int* const value_index,
      std::vector<c_float>* const q);

  void CalculateControlCostTerm(
      std::vector<std::vector<std::pair<c_int, c_float>>>* const columns, 
      int* const value_index);

  void AddStartPointConstraints(
      std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
      std::vector<c_float>* const lower_bounds, 
      std::vector<c_float>* const upper_bounds,
      int* const constraint_index);
  
  void AddKinematicsConstraints(
      std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
      std::vector<c_float>* const lower_bounds, 
      std::vector<c_float>* const upper_bounds,
      int* const constraint_index);

  void AddUpperAndLowerConstraints(
      std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
      std::vector<c_float>* const lower_bounds, 
      std::vector<c_float>* const upper_bounds,
      int* const constraint_index);

  void CalOriginSlState();
  void CalFinalSlState();

 private:
  double cost_ = std::numeric_limits<double>::infinity();
  pnc::VehicleConfig vehicle_config_;
  PathOptimizerWeights weights_;
  double init_speed_ = 0.0;
  double cruise_speed_ = 0.0;
  double half_width_ = 1.25;

  int num_of_knots_;

  double kappa_limit_;
  double dkappa_limit_;
  double boundary_start_s_;
  double boundary_delta_s_;
  double boundary_end_s_;
  double planning_end_s_;

  std::unordered_map<int, double> index_delta_s_map_;
  pnc::PathPoint planning_start_point_;
  pnc::FrenetFramePoint start_point_constraint_;
  std::vector<pnc::PathPoint> reference_points_;
  const PathBoundary* path_boundary_ = nullptr;

  std::vector<SlState> sl_states_;
};

} // namespace planning
} // namespace xju
