/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/path_optimizer/mass_point_model_optimizer.h"

#include <cmath>
#include "Eigen/Core"
#include "Eigen/Dense"

#include "common/vehicle_config/vehicle_config_provider.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include "common/math/math_utils.h"
#include "common/logger/logger.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "common/logger/logger.h"

namespace xju {
namespace planning {

using xju::pnc::TaskConfig;
using xju::pnc::VehicleConfigProvider;
using xju::pnc::VehicleStateProvider;
using xju::pnc::PathPoint;
using xju::pnc::TrajectoryPoint;
using xju::pnc_map::ReferenceLine;
using xju::pnc::kMathEpsilon;


MassPointModelOptimizer::MassPointModelOptimizer() 
    : pnc::QPSolver(5000) {}

void MassPointModelOptimizer::Init() {
  vehicle_config_ = VehicleConfigProvider::GetConfig();
  const double max_steer_angle = pnc::ToRadian(
      vehicle_config_.max_steer_angle() /
      vehicle_config_.steer_ratio());
  const double max_steer_angle_rate = pnc::ToRadian(
      vehicle_config_.max_steer_angle_rate() /
      vehicle_config_.steer_ratio());
  kappa_limit_ = std::tan(max_steer_angle) / 
      vehicle_config_.wheel_base();
  dkappa_limit_ = std::tan(max_steer_angle_rate) / 
      vehicle_config_.wheel_base();
  half_width_ = vehicle_config_.width() / 2.0;
}

void MassPointModelOptimizer::set_weights(
    const PathOptimizerWeights& weights) {
  weights_ = weights;
}

void MassPointModelOptimizer::SetCarSpeedInfo(
    const double init_speed, const double target_speed) {
  init_speed_ = init_speed;
  cruise_speed_ = target_speed;
  dkappa_limit_ = 2.0 * dkappa_limit_ / (init_speed_ + cruise_speed_ + kMathEpsilon);
}

bool MassPointModelOptimizer::OptimizePath(
    const ReferenceLine& reference_line,
    const PathPoint& start_point,
    const PathBoundary* const boundary) {
  // 1. Init local virables
  InitLocalVirables(start_point, boundary);

  // 2. calculate reference points
  CalculateReferencePoints(reference_line);

  // CalOriginSlState();

  // 3. check
  if (!Check()) {
    AERROR << "Input of MassPointModelOptimizer ERROR.";
    return false;
  }

  // 4. calculate start point constraint
  CalculateStartPointConstraint(start_point);

  // 5. construct optimizal problem and solve
  if (!Optimize()) {
    AERROR << "Optimize of MassPointModelOptimizer ERROR.";
    return false;
  }
  // CalFinalSlState();
  // if (!Optimize()) {
  //   AERROR << "Optimize of MassPointModelOptimizer ERROR.";
  //   return false;
  // }
  cost_ = std::fabs(obj_val_);
  return true;
}

bool MassPointModelOptimizer::GetOptimizePath(
    PathData* const path_data) {
  std::vector<pnc::FrenetFramePoint> frenet_points;
  for (int i = 0; i < num_of_knots_; ++i) {
    pnc::FrenetFramePoint pt;
    pt.set_s(reference_points_[i].s());
    pt.set_l(solution_[3 * i]);
    pt.set_heading_error(solution_[3 * i + 1]);
    pt.set_kappa(solution_[3 * i + 2]);

    if (i == 0) {
      pt.set_dkappa(planning_start_point_.dkappa());
    } else {
      pt.set_dkappa(solution_[3 * num_of_knots_ + i - 1]);
    }
    frenet_points.push_back(pt);
  }
  path_data->set_reference_points(reference_points_);
  if (!path_data->set_frenet_points(frenet_points)) {
    return false;
  }
  return true;
}

void MassPointModelOptimizer::CalculateStartPointConstraint(
  const PathPoint& start_point) {
  const PathPoint& reference_point = reference_points_.front();
  const double l = pnc::Distance(start_point, reference_point);
  const double dx = start_point.x() - reference_point.x();
  const double dy = start_point.y() - reference_point.y();
  const double cross = dy * std::cos(reference_point.theta()) - 
                       dx * std::sin(reference_point.theta());

  start_point_constraint_.set_l((cross > 0.0 ? 1.0 : -1.0) * l);
  start_point_constraint_.set_heading_error(
      pnc::NormalizeAngle(start_point.theta() - reference_point.theta()));
  start_point_constraint_.set_kappa(start_point.kappa());
}

void MassPointModelOptimizer::CalculateKernelAndOffset(
    std::vector<c_float>* const P_data, 
    std::vector<c_int>* const P_indices, 
    std::vector<c_int>* const P_indptr,
    std::vector<c_float>* const q) {

  P_data->clear(); P_indices->clear(); P_indptr->clear();
  q->resize(kernel_dim_, 0.0);
  std::vector<std::vector<std::pair<c_int, c_float>>> columns(kernel_dim_);
  int value_index = 0;

  CalculateStateCostTerm(&columns, &value_index, q);

  CalculateControlCostTerm(&columns, &value_index);

  for (int i = 0; i < num_of_knots_; ++i) {
    q->at(3 * i + 2) = -weights_.kappa() * reference_points_[i].kappa();
  }

  int ind_p = 0;
  for (int i = 0; i < kernel_dim_; ++i) {
    P_indptr->push_back(ind_p);
    for (const auto& row_data_pair : columns[i]) {
      P_data->push_back(row_data_pair.second);
      P_indices->push_back(row_data_pair.first);
      ++ind_p;
    }
  }
  P_indptr->push_back(ind_p);
}

void MassPointModelOptimizer::CalculateAffineConstraint(
    std::vector<c_float>* const A_data, 
    std::vector<c_int>* const A_indices, 
    std::vector<c_int>* const A_indptr, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds) {

  int num_of_constraints = 6 * num_of_knots_ - 3;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);
  std::vector<std::vector<std::pair<c_int, c_float>>> variables(kernel_dim_);
  int constraint_index = 0;

  // 1. start point constraint, 3
  AddStartPointConstraints(
      &variables, lower_bounds, upper_bounds, &constraint_index);

  // 2. equality constraint, vehicle model, 3(N - 1)
  AddKinematicsConstraints(
      &variables, lower_bounds, upper_bounds, &constraint_index);

  // 3. TODO::end state constraint, do not set now

  // 4. Virables upper and lower limits constraints: 3(N - 1)
  AddUpperAndLowerConstraints(
      &variables, lower_bounds, upper_bounds, &constraint_index);

  int ind_p = 0;
  for (int i = 0; i < kernel_dim_; ++i) {
    A_indptr->push_back(ind_p);
    for (const auto& variable_nz : variables[i]) {
      A_data->push_back(variable_nz.second);
      A_indices->push_back(variable_nz.first);
      ++ind_p;
    }
  }
  A_indptr->push_back(ind_p);
}

void MassPointModelOptimizer::CalculateReferencePoints(
    const ReferenceLine& reference_line) {
  reference_points_.clear();
  index_delta_s_map_.clear();

  const PathPoint& start_reference_pt = 
    reference_line.GetReferencePoint(
        planning_start_point_.x(), planning_start_point_.y());
  const PathPoint& end_reference_pt = 
    reference_line.GetReferencePoint(planning_end_s_);

  reference_points_.push_back(start_reference_pt);
  int index = 1;
  const double splice = FLAGS_gradually_discretization_splice;
  if (planning_end_s_ - start_reference_pt.s() > 150.0) {
    FLAGS_enable_gradually_discretization = true;
  }
  
  const auto& origin_reference_points = reference_line.reference_points();
  for (int i = 0, splice_index = 1; i < origin_reference_points.size();) {
    const auto& pt = origin_reference_points[i];
    if (pt.s() < start_reference_pt.s() + kMathEpsilon) {
      i += splice_index;
      continue;
    }
    if (pt.s() > planning_end_s_ - kMathEpsilon) {
      break;
    }

    double delta_s = pt.s() - reference_points_.back().s();
    index_delta_s_map_.insert(std::pair<int, double>(index, delta_s));
    ++index;

    reference_points_.push_back(pt);
    // gradually discretization
    if (FLAGS_enable_gradually_discretization && 
        pt.s() > start_reference_pt.s() + splice_index * splice) {
      ++splice_index;
    }
    i += splice_index;
  }

  double delta_s = end_reference_pt.s() - reference_points_.back().s();
  index_delta_s_map_.insert(std::pair<int, double>(index, delta_s));

  reference_points_.push_back(end_reference_pt);
  num_of_knots_ = reference_points_.size();
  kernel_dim_ = 4 * num_of_knots_ - 1;
}

void MassPointModelOptimizer::CalculateVehicleStateSpace(
    const double kappa, const double delta_s,
    Eigen::MatrixXd* const sys, Eigen::MatrixXd* const control, 
    Eigen::MatrixXd* const dis) {
  (*sys)(0, 0) = 1.0;
  (*sys)(0, 1) = delta_s;
  (*sys)(0, 2) = 0.0;
    
  (*sys)(1, 0) = -delta_s * kappa * kappa;
  (*sys)(1, 1) = 1.0;
  (*sys)(1, 2) = delta_s;

  (*sys)(2, 0) = 0.0;
  (*sys)(2, 1) = 0.0;
  (*sys)(2, 2) = 1.0;

  (*control)(0, 0) = 0.0;
  (*control)(1, 0) = 0.0;
  (*control)(2, 0) = delta_s;

  (*dis)(0) = 0.0;
  (*dis)(1) = -delta_s * kappa;
  (*dis)(2) = 0.0;
}

void MassPointModelOptimizer::CalculateVehicleStateSpace(
    const int index,
    Eigen::MatrixXd* const sys, Eigen::MatrixXd* const control, 
    Eigen::MatrixXd* const dis) {

  const SlState& state = sl_states_.at(index);
  const SlState& state_next = sl_states_.at(index + 1);
  const double kappa_ref = reference_points_[index].kappa();

  Eigen::Matrix3d df_x(Eigen::Matrix3d::Zero());
  df_x << -kappa_ref * std::tan(state.d_theta), 
              (1 - kappa_ref * state.l) * (1 + std::pow(std::tan(state.d_theta), 2)), 0,
          -kappa_ref * state.kappa / std::cos(state.d_theta), 
              (1 - kappa_ref * state.l) * state.kappa * std::tan(state.d_theta) / std::cos(state.d_theta), 
              (1 - kappa_ref * state.l) / std::cos(state.d_theta),
          0, 0, 0;

  Eigen::Matrix<double, 3, 1> df_u;
  df_u << 0, 0, 1;

  const double ds = reference_points_[index + 1].s() - reference_points_[index].s();
  const auto A = ds * df_x + Eigen::Matrix3d::Identity();
  const auto B = ds * df_u;

  const double u_input = (state_next.kappa - state.kappa) / ds;
  Eigen::Matrix<double, 3, 1> f_x_u;
  f_x_u << (1 - kappa_ref * state.l) * std::tan(state.d_theta),
           (1 - kappa_ref * state.l) * state.kappa / std::cos(state.d_theta) - kappa_ref,
            u_input; // TODO: use dk directly.
        
  Eigen::Matrix<double, 3, 1> x_vec;
  x_vec << state.l, state.d_theta, state.kappa;
  
  *sys = A;
  *control = B;
  *dis = ds *(f_x_u - df_x * x_vec - df_u * u_input);
}

std::pair<double, double> 
MassPointModelOptimizer::CalculateBoundByS(
    const std::vector<std::pair<double, double>>& bounds,
    const double s) {
  double delta_s = s - boundary_start_s_;
  size_t num_of_bound = bounds.size();
  size_t front_index = std::fmax(std::fmin(
      std::floor(delta_s / boundary_delta_s_), num_of_bound - 1), 0);
  size_t back_index = std::fmin(std::fmax(
      std::ceil(delta_s / boundary_delta_s_), 0), num_of_bound - 1);
  
  // conservative strategy
  std::pair<double, double> bound;
  bound.first = std::fmax(bounds[front_index].first, bounds[back_index].first) + half_width_;
  bound.second = std::fmin(bounds[front_index].second, bounds[back_index].second) - half_width_;
  return bound;
}

void MassPointModelOptimizer::InitLocalVirables(
    const pnc::PathPoint& start_point,
    const PathBoundary* const boundary) {
  CHECK(boundary);
  path_boundary_ = boundary;
  boundary_start_s_ = boundary->start_s();
  boundary_delta_s_ = boundary->delta_s();
  boundary_end_s_ = boundary_start_s_ + boundary->Length();

  planning_start_point_ = start_point;
  planning_end_s_ = boundary_end_s_ - 
      vehicle_config_.wheel_base() -
      vehicle_config_.front_overhang_length();
}

bool MassPointModelOptimizer::Check() {
  if (boundary_start_s_ > boundary_end_s_ - kMathEpsilon) {
    AERROR << "Start_s >= End_s of boundary";
    return false;
  }
  if (reference_points_.size() < 2) {
    AERROR << "Reference Points size < 2";
    return false;
  }
  // if (equilibrium_points_.size() != reference_points_.size()) {
  //   AERROR << "Reference Points size is not equal to Equilibrium Points.";
  //   return false;
  // }
  return true;
}

void MassPointModelOptimizer::SetWarmStartX() {
  start_x_.resize(kernel_dim_, 0.0);
  for (int i = 0; i < kernel_dim_ && i < solution_.size(); ++i) {
    start_x_.push_back(static_cast<c_float>(solution_[i]));
  }
}

void MassPointModelOptimizer::CalculateStateCostTerm(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const columns, 
    int* const value_index,
    std::vector<c_float>* const q) {
  const double speed = (cruise_speed_ + init_speed_) / 2.0;
  const double weight_theta = weights_.theta() * std::fmax(speed * speed, 5.0);
  for (int i = 0; i < num_of_knots_; ++i) {
    columns->at(3 * i).emplace_back(*value_index, weights_.l());
    ++*value_index;
    columns->at(3 * i + 1).emplace_back(*value_index, weight_theta);
    ++*value_index;
    columns->at(3 * i + 2).emplace_back(*value_index, weights_.kappa());
    ++*value_index;
  }
}

void MassPointModelOptimizer::CalculateControlCostTerm(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const columns, 
    int* const value_index) {
  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    columns->at(3 * num_of_knots_ + i).emplace_back(*value_index, weights_.dkappa());
    ++*value_index;
  }
}

void MassPointModelOptimizer::AddStartPointConstraints(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds,
    int* const constraint_index) {
  // start point constraint, lateral offset
  variables->at(0).emplace_back(*constraint_index, 1.0);
  lower_bounds->at(*constraint_index) = start_point_constraint_.l();
  upper_bounds->at(*constraint_index) = start_point_constraint_.l();
  ++*constraint_index;
  // start point constraint, car heading
  variables->at(1).emplace_back(*constraint_index, 1.0);
  lower_bounds->at(*constraint_index) = start_point_constraint_.heading_error();
  upper_bounds->at(*constraint_index) = start_point_constraint_.heading_error();
  ++*constraint_index;

  // start point constraint, kappa
  variables->at(2).emplace_back(*constraint_index, 1.0 );
  lower_bounds->at(*constraint_index) = start_point_constraint_.kappa();
  upper_bounds->at(*constraint_index) = start_point_constraint_.kappa();
  ++*constraint_index;
}

void MassPointModelOptimizer::AddKinematicsConstraints(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds,
    int* const constraint_index) {
  Eigen::MatrixXd sys(3, 3), control(3, 1), dis(3, 1);
  for (int i = 1; i < num_of_knots_; ++i) {
    double kappa = reference_points_[i].kappa();
    double delta_s = index_delta_s_map_.find(i)->second;
    CalculateVehicleStateSpace(kappa, delta_s, &sys, &control, &dis);

    // CalculateVehicleStateSpace(i - 1, &sys, &control, &dis);

    // error l
    variables->at(3 * i - 3).emplace_back(*constraint_index, sys(0, 0));
    variables->at(3 * i - 2).emplace_back(*constraint_index, sys(0, 1));
    variables->at(3 * i - 1).emplace_back(*constraint_index, sys(0, 2));
    variables->at(3 * i).emplace_back(*constraint_index, -1.0);
    variables->at(3 * num_of_knots_ + i - 1).emplace_back(*constraint_index, control(0, 0));
    lower_bounds->at(*constraint_index) = -dis(0, 0);
    upper_bounds->at(*constraint_index) = -dis(0, 0);
    ++*constraint_index;

    // error theta
    variables->at(3 * i - 3).emplace_back(*constraint_index, sys(1, 0));
    variables->at(3 * i - 2).emplace_back(*constraint_index, sys(1, 1));
    variables->at(3 * i - 1).emplace_back(*constraint_index, sys(1, 2));
    variables->at(3 * i + 1).emplace_back(*constraint_index, -1.0);
    variables->at(3 * num_of_knots_ + i - 1).emplace_back(*constraint_index, control(1, 0));
    lower_bounds->at(*constraint_index) = -dis(1, 0);
    upper_bounds->at(*constraint_index) = -dis(1, 0);
    ++*constraint_index;

    // kappa
    variables->at(3 * i - 3).emplace_back(*constraint_index, sys(2, 0));
    variables->at(3 * i - 2).emplace_back(*constraint_index, sys(2, 1));
    variables->at(3 * i - 1).emplace_back(*constraint_index, sys(2, 2));
    variables->at(3 * i + 2).emplace_back(*constraint_index, -1.0);
    variables->at(3 * num_of_knots_ + i - 1).emplace_back(*constraint_index, control(2, 0));
    lower_bounds->at(*constraint_index) = -dis(2, 0);
    upper_bounds->at(*constraint_index) = -dis(2, 0);
    ++*constraint_index;
  }
}

void MassPointModelOptimizer::AddUpperAndLowerConstraints(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds,
    int* const constraint_index) {
  // 3(N - 1)
  for (int i = 1; i < num_of_knots_; ++i) {
     // simple model, l constraints
    std::pair<double, double> bound = 
        CalculateBoundByS(path_boundary_->path_boundary(), reference_points_[i].s());
    variables->at(3 * i).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = bound.first;
    upper_bounds->at(*constraint_index) = bound.second;
    ++*constraint_index;
    
    // Car heading constraints
    variables->at(3 * i + 1).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = -FLAGS_heading_error_bound;
    upper_bounds->at(*constraint_index) = FLAGS_heading_error_bound;
    ++*constraint_index;
    
    // kappa
    variables->at(3 * i + 2).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = -kappa_limit_;
    upper_bounds->at(*constraint_index) = kappa_limit_;
    ++*constraint_index;
  }
}

void MassPointModelOptimizer::CalOriginSlState() {
  sl_states_.resize(num_of_knots_);
  for (int i = 0; i < num_of_knots_; ++i) {
    sl_states_[i].s = reference_points_[i].s();
    sl_states_[i].l = 0.0;
    sl_states_[i].d_theta = 0.0;
    sl_states_[i].kappa = reference_points_[i].kappa();
  }
}

void MassPointModelOptimizer::CalFinalSlState() {
  for (int i = 0; i < num_of_knots_; ++i) {
    sl_states_[i].l = solution_[3 * i];
    sl_states_[i].d_theta = solution_[3 * i + 1];
    sl_states_[i].kappa = solution_[3 * i + 2];
  }
}

} // namespace planning
} // namespace xju