/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <array>
#include <vector>
#include <unordered_map>
#include <limits>

#include "common/third_party/hpipm_cpp/hpipm_cpp.h"
#include "pnc_map/reference_line.h"
#include "planning/common/path/path_boundary.h"
#include "planning/common/path/path_data.h"
#include "task_config.pb.h"
#include "vehicle_config.pb.h"

namespace xju {
namespace planning {

class VehicleModelOptimizer {
 public:
  struct Partial {
    double l;
    double theta;
    double beta;
    double kappa;
    double constant;
  };

  enum CornerType {
    FRONT_CORNER = 1,
    FRONT_AXLE = 2,
    REAR_AXLE = 3,
    REAR_CORNER = 4,
  };

  VehicleModelOptimizer() = default;
  ~VehicleModelOptimizer() = default;

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

  void SetTarget(const double x, const double y, const double theta);

 private:
  void InitLocalVirables(
      const pnc::PathPoint& start_point,
      const PathBoundary* const boundary);

  void ResetTargetInfo();

  void CalculateTargetInfo(const pnc_map::ReferenceLine& reference_line);

  void CalculateStartPointConstraint(
      const pnc::PathPoint& start_point);

  void CalculateReferencePoints(
      const pnc_map::ReferenceLine& reference_line);

  bool Check();
  
  bool Optimize();

  void CalculateVehicleStateSpace(
      const double kappa, const double delta_s,
      Eigen::MatrixXd* const sys, Eigen::MatrixXd* const control, 
      Eigen::VectorXd* const dis);

  std::pair<double, double> CalculateBoundByS(
      const std::vector<std::pair<double, double>>& bounds,
      const double s);

  double CalculateCarStationByIndex(
      const size_t index, const double length);

  Partial CalculateCarEdgeLaterOffsetCoeff(
      const size_t index, const CornerType& type);

  void GetCarCornerParams(
      const CornerType& type,
      double* const l, double* const w, double* const d);

  void SetIpmSolverSettings(
      pnc::hpipm::OcpQpIpmSolverSettings* const solver_settings);

  void SetWarmStartValue();

  VehicleModelOptimizer::Partial CalculateCarEdgeLaterOffsetCoeffTest(
      const CornerType& type, const double kappa);

 private:
  bool status_ = false;
  std::string boundary_label_;
  double cost_ = std::numeric_limits<double>::infinity();
  pnc::VehicleConfig vehicle_config_;
  PathOptimizerWeights weights_;
  double half_width_ = 1.25;
  double init_speed_ = 0.0;
  double cruise_speed_ = 0.0;

  int num_of_knots_;
  double kappa_limit_;
  double dkappa_limit_;
  double boundary_start_s_;
  double boundary_delta_s_;
  double boundary_end_s_;
  double planning_end_s_;

  // target info
  bool has_target_ = false;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
  double target_theta_ = 0.0;
  double target_l_ = 0.0;
  double target_theta_error_ = 0.0;
  int target_index_ = std::numeric_limits<int>::max();

  std::unordered_map<int, double> index_delta_s_map_;
  pnc::PathPoint planning_start_point_;
  std::vector<pnc::PathPoint> reference_points_;
  const PathBoundary* path_boundary_ = nullptr;

  std::vector<pnc::hpipm::OcpQpSolution> solution_;
  Eigen::VectorXd init_state_;
  // std::array<double, 4> scale_factor_ = {{1.0, 100.0, 100.0, 100.0}};
  std::vector<pnc::FrenetFramePoint> frenet_points_;
  // vehicle
  std::vector<pnc::hpipm::OcpQp> vehicle_ocp_qps_;

  Eigen::MatrixXd vehicle_sys_ = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd vehicle_control_ = Eigen::MatrixXd::Zero(4, 1);
  Eigen::VectorXd vehicle_dis_ = Eigen::VectorXd::Zero(4);

  Eigen::MatrixXd vehicle_Q_ = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd vehicle_R_ = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd vehicle_S_ = Eigen::MatrixXd::Zero(1, 4);
  Eigen::VectorXd vehicle_q_ = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd vehicle_r_ = Eigen::VectorXd::Zero(1);

  Eigen::MatrixXd vehicle_C_ = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd vehicle_D_ = Eigen::MatrixXd::Zero(4, 1);
  Eigen::VectorXd vehicle_lg_ = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd vehicle_ug_ = Eigen::VectorXd::Zero(4);

  Eigen::MatrixXd vehicle_C_lane_ = Eigen::MatrixXd::Zero(2, 4);
  Eigen::MatrixXd vehicle_D_lane_ = Eigen::MatrixXd::Zero(2, 1);
  Eigen::VectorXd vehicle_lg_lane_ = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd vehicle_ug_lane_ = Eigen::VectorXd::Zero(2);

  std::vector<pnc::hpipm::OcpQp> ocp_qps_;

  Eigen::MatrixXd sys_ = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd control_ = Eigen::MatrixXd::Zero(3, 1);
  Eigen::VectorXd dis_ = Eigen::VectorXd::Zero(3);

  Eigen::MatrixXd Q_ = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd R_ = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd S_ = Eigen::MatrixXd::Zero(1, 3);
  Eigen::VectorXd q_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd r_ = Eigen::VectorXd::Zero(1);

  Eigen::MatrixXd C_ = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd D_ = Eigen::MatrixXd::Zero(3, 1);
  Eigen::VectorXd lg_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd ug_ = Eigen::VectorXd::Zero(3);

  Eigen::MatrixXd C_lane_ = Eigen::MatrixXd::Zero(1, 3);
  Eigen::MatrixXd D_lane_ = Eigen::MatrixXd::Zero(1, 1);
  Eigen::VectorXd lg_lane_= Eigen::VectorXd::Zero(1);
  Eigen::VectorXd ug_lane_ = Eigen::VectorXd::Zero(1);

  Eigen::VectorXd negative_kl_ = Eigen::VectorXd::Zero(1);
  Eigen::VectorXd positive_kl_ = Eigen::VectorXd::Zero(1);

  // slack matrxi
  Eigen::MatrixXd Zl_ = Eigen::MatrixXd::Zero(2, 2);
  Eigen::MatrixXd Zu_ = Eigen::MatrixXd::Zero(2, 2);
  Eigen::VectorXd zl_ = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd zu_ = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lls_ = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd lus_ = Eigen::VectorXd::Zero(2);
};

} // namespace planning
} // namespace xju
