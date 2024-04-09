/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <array>
#include <vector>
#include <unordered_map>
#include <limits>

#include "common/math/qp_solver/qp_solver.h"
#include "pnc_map/reference_line.h"
#include "planning/common/path/path_boundary.h"
#include "planning/common/path/path_data.h"
#include "task_config.pb.h"
#include "vehicle_config.pb.h"
#include "Eigen/Core"

namespace xju {
namespace planning {

class VehicleModelOptimizerOSQP : public pnc::QPSolver {
 public:
  struct Partial {
    double l;
    double theta;
    double beta;
    double kappa;
    double constant;
  };

  struct EquilibriumPoint {
    double l;
    double theta;
    double beta;
    double kappa;
  };

  enum CornerType {
    FRONT_CORNER = 1,
    FRONT_AXLE = 2,
    REAR_AXLE = 3,
    REAR_CORNER = 4,
  };

 public:
  VehicleModelOptimizerOSQP();
  ~VehicleModelOptimizerOSQP() = default;

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

  bool status() const {
    return status_;
  }

  const std::string& boundary_label() const {
    return boundary_label_;
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

  Partial CalculateCarEdgeLaterOffsetCoeff(
      const size_t index, const CornerType& type);

  void GetCarCornerParams(
      const CornerType& type,
      double* const l, double* const w, double* const d);

  double CalculateBestLaterOffsetParam(const size_t index);

  void CalculateVehicleStateSpace(
      const double kappa, const double delta_s,
      Eigen::MatrixXd* const sys, Eigen::MatrixXd* const control, 
      Eigen::MatrixXd* const dis);

  double CalculateCarStationByIndex(
      const size_t index, const double length);

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

  void AddCarEdgeInLaneAndNoCollision(
      std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
      std::vector<c_float>* const lower_bounds, 
      std::vector<c_float>* const upper_bounds,
      int* const constraint_index);

  void AddCarEdgeNoCollision(
      std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
      std::vector<c_float>* const lower_bounds, 
      std::vector<c_float>* const upper_bounds,
      int* const constraint_index);

 private:
  bool status_ = false;
  std::string boundary_label_;
  double cost_ = std::numeric_limits<double>::infinity();
  pnc::VehicleConfig vehicle_config_;
  PathOptimizerWeights weights_;
  double init_speed_ = 0.0;
  double cruise_speed_ = 0.0;

  int num_of_knots_;
  int num_of_state_;

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
  std::vector<EquilibriumPoint> equilibrium_points_;

  std::array<double, 4> scale_factor_ = {{1.0, 100.0, 100.0, 100.0}};
};

} // namespace planning
} // namespace xju
