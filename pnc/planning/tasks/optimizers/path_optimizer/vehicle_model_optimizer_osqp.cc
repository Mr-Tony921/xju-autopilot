/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/path_optimizer/vehicle_model_optimizer_osqp.h"

#include <cmath>

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


VehicleModelOptimizerOSQP::VehicleModelOptimizerOSQP() 
    : pnc::QPSolver(5000) {}

void VehicleModelOptimizerOSQP::Init() {
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
}

void VehicleModelOptimizerOSQP::set_weights(
    const PathOptimizerWeights& weights) {
  weights_ = weights;
}

void VehicleModelOptimizerOSQP::SetCarSpeedInfo(
    const double init_speed, const double target_speed) {
  init_speed_ = init_speed;
  cruise_speed_ = target_speed;
}

bool VehicleModelOptimizerOSQP::OptimizePath(
    const ReferenceLine& reference_line,
    const PathPoint& start_point,
    const PathBoundary* const boundary) {
  // 1. Init local virables
  InitLocalVirables(start_point, boundary);

  // 2. calculate reference points
  CalculateReferencePoints(reference_line);

  // 3. check
  if (!Check()) {
    AERROR << "Input of VehicleModelOptimizerOSQP ERROR.";
    return false;
  }

  // 4. calculate start point constraint
  CalculateStartPointConstraint(start_point);

  // 5. construct optimizal problem and solve
  if (!Optimize()) {
    AERROR << "Optimize of VehicleModelOptimizerOSQP ERROR.";
    return false;
  }
  cost_ = obj_val_;
  status_ = true;
  return true;
}

bool VehicleModelOptimizerOSQP::GetOptimizePath(
    PathData* const path_data) {

  std::vector<pnc::FrenetFramePoint> frenet_points;
  for (int i = 0; i < num_of_knots_; ++i) {
    pnc::FrenetFramePoint pt;
    pt.set_s(reference_points_[i].s());
    pt.set_l(solution_[num_of_state_ * i] / scale_factor_[0]);
    pt.set_heading_error(solution_[num_of_state_ * i + 1] / scale_factor_[1]);
    if (i < num_of_knots_ - 1) {
      pt.set_kappa(solution_[num_of_state_ * num_of_knots_ + i] / scale_factor_[3]);
    } else {
      pt.set_kappa(solution_.back() / scale_factor_[3]);
    }
    frenet_points.push_back(pt);
    // ADEBUG << "l = " << pt.l();
    // ADEBUG << "heading_error = " << pt.heading_error();
    // ADEBUG << "kappa = " << pt.kappa();
    // ADEBUG << "----------------------------------------";
  }
  path_data->set_reference_points(reference_points_);
  if (!path_data->set_frenet_points(frenet_points)) {
    return false;
  }
  return true;
}

void VehicleModelOptimizerOSQP::CalculateStartPointConstraint(
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

void VehicleModelOptimizerOSQP::CalculateKernelAndOffset(
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

void VehicleModelOptimizerOSQP::CalculateAffineConstraint(
    std::vector<c_float>* const A_data, 
    std::vector<c_int>* const A_indices, 
    std::vector<c_int>* const A_indptr, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds) {

  int num_of_constraints = 12 * num_of_knots_ - 10;
  num_of_constraints = 8 * num_of_knots_ - 7;
  // AINFO << "num_of_constraints = " << num_of_constraints;
  lower_bounds->resize(num_of_constraints);
  upper_bounds->resize(num_of_constraints);
  std::vector<std::vector<std::pair<c_int, c_float>>> variables(kernel_dim_);
  int constraint_index = 0;

  // 1. start point constraint, 4 or 3
  AddStartPointConstraints(
      &variables, lower_bounds, upper_bounds, &constraint_index);

  // 2. equality constraint, vehicle model, 3(N - 1) or 2(N - 1)
  AddKinematicsConstraints(
      &variables, lower_bounds, upper_bounds, &constraint_index);

  // 3. TODO::end state constraint, do not set now

  // 4. Virables upper and lower limits constraints: 5N - 7 or 4N - 6
  AddUpperAndLowerConstraints(
      &variables, lower_bounds, upper_bounds, &constraint_index);

  // 5. trie position constraints, in lane and no obstalce collision: N - 1
  AddCarEdgeInLaneAndNoCollision(
      &variables, lower_bounds, upper_bounds, &constraint_index);

  // 6. corner position constraints, no obstalce collision: N - 1
  AddCarEdgeNoCollision(
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

void VehicleModelOptimizerOSQP::CalculateReferencePoints(
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
  num_of_state_ = 2;
  kernel_dim_ = 3 * num_of_knots_ - 1;
  // ADEBUG << "FLAGS_enable_gradually_discretization: " << FLAGS_enable_gradually_discretization;
  // ADEBUG << "reference length is: " << reference_points_.back().s() - reference_points_.front().s();
  // ADEBUG << "num_of_knots_ = " << num_of_knots_;
  // ADEBUG << "kernel_dim_ = " << kernel_dim_;
}

VehicleModelOptimizerOSQP::Partial 
VehicleModelOptimizerOSQP::CalculateCarEdgeLaterOffsetCoeff(
    const size_t index, const CornerType& type) {
  const double kappa = reference_points_[index].kappa();
  double l, w, d;
  GetCarCornerParams(type, &l, &w, &d);
  
  Partial partial;
  if (std::fabs(kappa) < kMathEpsilon) {
    partial.l = 1.0;
    partial.theta = l;
    partial.beta = 0.0;
    partial.kappa = 0.0;
    partial.constant = -w;
    return partial;
  }

  if (kappa > 0.0) {
    partial.l = 1.0 / kappa / std::sqrt(d * d + 1.0 / (kappa * kappa));
    partial.theta = d / kappa / std::sqrt(d * d + 1.0 / (kappa * kappa));
    partial.beta = 0.0;
    partial.kappa = 0.0;
    partial.constant = -std::sqrt(d * d + 1.0 / (kappa * kappa)) + 1.0 / kappa;
  } else {
    partial.l = -1.0 / kappa / std::sqrt(d * d + 1.0 / (kappa * kappa));
    partial.theta = -d / kappa / std::sqrt(d * d + 1.0 / (kappa * kappa));
    partial.beta = 0.0;
    partial.kappa = 0.0;
    partial.constant = std::sqrt(d * d + 1.0 / (kappa * kappa)) + 1.0 / kappa;
  }
  return partial;
}

void VehicleModelOptimizerOSQP::GetCarCornerParams(
    const CornerType& type,
    double* const l, double* const w, double* const d) {
  const double wheel_base = vehicle_config_.wheel_base();
  const double front_oh = vehicle_config_.front_overhang_length();
  const double rear_oh = vehicle_config_.back_overhang_length();
  
  switch (type) {
    // simple model
    case CornerType::FRONT_CORNER:
      *l = wheel_base + front_oh;
      *w = 0.0;
      *d = std::hypot(*l, *w); 
      break;
    case CornerType::FRONT_AXLE:
      *l = wheel_base;
      *w = 0.0;
      *d = std::hypot(*l, *w); 
      break;
    case CornerType::REAR_AXLE:
      *l = 0.0;
      *w = 0.0;
      *d = std::hypot(*l, *w); 
      break;
    case CornerType::REAR_CORNER:
      *l = -rear_oh;
      *w = 0.0;
      *d = std::hypot(*l, *w); 
      break;
    default:
      break;
  }
}

double VehicleModelOptimizerOSQP::CalculateBestLaterOffsetParam(
    const size_t index) {
  // use experience data
  return weights_.area();
}

void VehicleModelOptimizerOSQP::CalculateVehicleStateSpace(
    const double kappa, const double delta_s,
    Eigen::MatrixXd* const sys, Eigen::MatrixXd* const control, 
    Eigen::MatrixXd* const dis) {
  
  const double rear_axle_to_hitch = 
      vehicle_config_.rear_axle_to_hitch();
  
  // in highway, let bate = 0, so donot need equilibrium point
  (*sys)(0, 0) = 1.0;
  (*sys)(0, 1) = delta_s;
  (*sys)(0, 2) = 0.0;
  (*sys)(1, 0) = -delta_s * kappa * kappa;
  (*sys)(1, 1) = 1.0;
  (*sys)(1, 2) = 0.0;
  (*sys)(2, 0) = delta_s * kappa * kappa;
  (*sys)(2, 1) = 0.0;
  (*sys)(2, 2) = 1.0 - delta_s;

  (*control)(0, 0) = 0.0;
  (*control)(1, 0) = delta_s;
  (*control)(2, 0) = -delta_s * rear_axle_to_hitch;

  (*dis)(0, 0) = 0.0;
  (*dis)(1, 0) = -delta_s * kappa;
  (*dis)(2, 0) = 0.0;
}

double VehicleModelOptimizerOSQP::CalculateCarStationByIndex(
    const size_t index, const double length) {
  const auto& reference_point = reference_points_[index];
  // const auto& equilibrium_point = equilibrium_points_[index];
  return reference_point.s() + length;
}

std::pair<double, double> 
VehicleModelOptimizerOSQP::CalculateBoundByS(
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
  bound.first = std::fmax(bounds[front_index].first, bounds[back_index].first);
  bound.second = std::fmin(bounds[front_index].second, bounds[back_index].second);
  return bound;
}

void VehicleModelOptimizerOSQP::InitLocalVirables(
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
  cost_ = std::numeric_limits<double>::infinity();
  status_ = false;
  boundary_label_ = boundary->label();
}

bool VehicleModelOptimizerOSQP::Check() {
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

void VehicleModelOptimizerOSQP::SetWarmStartX() {
  start_x_.resize(kernel_dim_, 0.0);
  for (int i = 0; i < kernel_dim_ && i < solution_.size(); ++i) {
    start_x_.push_back(static_cast<c_float>(solution_[i]));
  }
}

void VehicleModelOptimizerOSQP::CalculateStateCostTerm(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const columns, 
    int* const value_index,
    std::vector<c_float>* const q) {
  for (int i = 0; i < num_of_knots_; ++i) {
    columns->at(2 * i).emplace_back(
        *value_index, weights_.l() / scale_factor_[0] / scale_factor_[0]);
    ++*value_index;
    columns->at(2 * i + 1).emplace_back(
        *value_index, weights_.theta() / scale_factor_[1] / scale_factor_[1]);
    ++*value_index;
  }
}

void VehicleModelOptimizerOSQP::CalculateControlCostTerm(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const columns, 
    int* const value_index) {
  const double scale_factor = scale_factor_[3] * scale_factor_[3];
  if (num_of_knots_ <= 2) {
    for (int i = 0; i < num_of_knots_ - 1; ++i) {
      columns->at(num_of_state_ * num_of_knots_ + i).emplace_back(
          *value_index, (weights_.kappa()) / scale_factor);
      ++*value_index;
    }
    return;
  }
  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    if (i == 0) {
      columns->at(num_of_state_ * num_of_knots_ + i).emplace_back(
          *value_index, (weights_.kappa() + weights_.dkappa()) / scale_factor);
      columns->at(num_of_state_ * num_of_knots_ + i + 1).emplace_back(
          *value_index, -weights_.dkappa() / scale_factor);
      ++*value_index;
    } else if (i == num_of_knots_ - 2) {
      columns->at(num_of_state_ * num_of_knots_ + i - 1).emplace_back(
          *value_index, -weights_.dkappa() / scale_factor);
      columns->at(num_of_state_ * num_of_knots_ + i).emplace_back(
          *value_index, (weights_.kappa() + weights_.dkappa()) / scale_factor);
      ++*value_index;
    } else {
      columns->at(num_of_state_ * num_of_knots_ + i - 1).emplace_back(
          *value_index, -weights_.dkappa() / scale_factor);
      columns->at(num_of_state_ * num_of_knots_ + i).emplace_back(
          *value_index, (weights_.kappa() + 2 * weights_.dkappa()) / scale_factor);
      columns->at(num_of_state_ * num_of_knots_ + i + 1).emplace_back(
          *value_index, -weights_.dkappa() / scale_factor);
      ++*value_index;
    }
  }
}

void VehicleModelOptimizerOSQP::AddStartPointConstraints(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds,
    int* const constraint_index) {
  // start point constraint, lateral offset
  variables->at(0).emplace_back(*constraint_index, 1.0);
  lower_bounds->at(*constraint_index) = start_point_constraint_.l() * scale_factor_[0];
  upper_bounds->at(*constraint_index) = start_point_constraint_.l() * scale_factor_[0];
  ++*constraint_index;
  // start point constraint, car heading
  variables->at(1).emplace_back(*constraint_index, 1.0);
  lower_bounds->at(*constraint_index) = 
      start_point_constraint_.heading_error() * scale_factor_[1];
  upper_bounds->at(*constraint_index) = 
      start_point_constraint_.heading_error() * scale_factor_[1];
  ++*constraint_index;

  // start point constraint, kappa
  variables->at(num_of_state_ * num_of_knots_).emplace_back(*constraint_index, 1.0 );
  lower_bounds->at(*constraint_index) = 
      start_point_constraint_.kappa() * scale_factor_[3];
  upper_bounds->at(*constraint_index) = 
      start_point_constraint_.kappa() * scale_factor_[3];
  ++*constraint_index;
}

void VehicleModelOptimizerOSQP::AddKinematicsConstraints(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds,
    int* const constraint_index) {
  Eigen::MatrixXd sys(3, 3), control(3, 1), dis(3, 1);
  for (int i = 1; i < num_of_knots_; ++i) {
    double kappa = reference_points_[i].kappa();
    double delta_s = index_delta_s_map_.find(i)->second;
    CalculateVehicleStateSpace(kappa, delta_s, &sys, &control, &dis);

    // error l
    variables->at(2 * i - 2).emplace_back(*constraint_index,
                                          sys(0, 0) * scale_factor_[1]);
    variables->at(2 * i - 1).emplace_back(*constraint_index,
                                          sys(0, 1) * scale_factor_[0]);
    variables->at(2 * i).emplace_back(*constraint_index,
                                      -1.0 * scale_factor_[1]);
    // control(0, 0) = 0
    // variables->at(2 * num_of_knots_ + i - 1).emplace_back(*constraint_index,
    // control(0, 0));
    lower_bounds->at(*constraint_index) =
        -dis(0, 0) * scale_factor_[0] * scale_factor_[1];
    upper_bounds->at(*constraint_index) =
        -dis(0, 0) * scale_factor_[0] * scale_factor_[1];
    ++*constraint_index;
    // error theta
    variables->at(2 * i - 2).emplace_back(
        *constraint_index, sys(1, 0) * scale_factor_[1] * scale_factor_[3]);
    variables->at(2 * i - 1).emplace_back(
        *constraint_index, sys(1, 1) * scale_factor_[0] * scale_factor_[3]);
    variables->at(2 * i + 1).emplace_back(
        *constraint_index, -1.0 * scale_factor_[0] * scale_factor_[3]);
    variables->at(2 * num_of_knots_ + i - 1)
        .emplace_back(*constraint_index,
                      control(1, 0) * scale_factor_[0] * scale_factor_[1]);
    lower_bounds->at(*constraint_index) =
        -dis(1, 0) * scale_factor_[0] * scale_factor_[1] * scale_factor_[3];
    upper_bounds->at(*constraint_index) =
        -dis(1, 0) * scale_factor_[0] * scale_factor_[1] * scale_factor_[3];
    ++*constraint_index;
  }
}

void VehicleModelOptimizerOSQP::AddUpperAndLowerConstraints(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds,
    int* const constraint_index) {

  for (int i = 1; i < num_of_knots_; ++i) {
     // simple model, l constraints: N - 1
    std::pair<double, double> bound = 
        CalculateBoundByS(path_boundary_->path_boundary(), reference_points_[i].s());
    variables->at(num_of_state_ * i).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = bound.first * scale_factor_[0];
    upper_bounds->at(*constraint_index) = bound.second * scale_factor_[0];
    ++*constraint_index;
    
    // Car heading constraints: N - 1
    variables->at(num_of_state_ * i + 1).emplace_back(*constraint_index, 1.0);
    lower_bounds->at(*constraint_index) = 
        -FLAGS_heading_error_bound * scale_factor_[1];
    upper_bounds->at(*constraint_index) = 
        FLAGS_heading_error_bound * scale_factor_[1];
    ++*constraint_index;
  }

  const double v = (VehicleStateProvider::speed() + cruise_speed_) / 2.0 + 
      pnc::kMathEpsilon;
  if (num_of_knots_ > 2) {
    for (int i = 1; i < num_of_knots_ - 1; ++i) {
      // kappa constraints: N - 2
      variables->at(num_of_state_ * num_of_knots_ + i).emplace_back(*constraint_index, 1.0);
      lower_bounds->at(*constraint_index) = -kappa_limit_  * scale_factor_[3];
      upper_bounds->at(*constraint_index) = kappa_limit_ * scale_factor_[3];
      ++*constraint_index;
 
      // dkappa constraints, convert to time domain: N - 2
      variables->at(num_of_state_ * num_of_knots_ + i - 1).emplace_back(*constraint_index, -1.0);
      variables->at(num_of_state_ * num_of_knots_ + i).emplace_back(*constraint_index, 1.);
      double delta_s = index_delta_s_map_.find(i)->second;
      lower_bounds->at(*constraint_index) = -dkappa_limit_ * delta_s / v  * scale_factor_[3];
      upper_bounds->at(*constraint_index) = dkappa_limit_ * delta_s / v * scale_factor_[3];
      ++*constraint_index;
    }
  }
}

void VehicleModelOptimizerOSQP::AddCarEdgeInLaneAndNoCollision(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds,
    int* const constraint_index) {
  double s_hat;
  Partial patial;
  std::pair<double, double> bound;
  // tire position constraints, in lane and no collision
  const double wheel_base = vehicle_config_.wheel_base();
  for (int i = 1; i < num_of_knots_; ++i) {
    s_hat = CalculateCarStationByIndex(i, wheel_base);
    patial = CalculateCarEdgeLaterOffsetCoeff(i, CornerType::FRONT_AXLE);
    bound = CalculateBoundByS(path_boundary_->path_boundary(), s_hat);
    
    variables->at(num_of_state_ * i).emplace_back(
        *constraint_index, patial.l  * scale_factor_[1]);
    variables->at(num_of_state_ * i + 1).emplace_back(
        *constraint_index, patial.theta * scale_factor_[0]);
    lower_bounds->at(*constraint_index) = 
        (bound.first - patial.constant) * scale_factor_[0] * scale_factor_[1];
    upper_bounds->at(*constraint_index) = 
        (bound.second - patial.constant) * scale_factor_[0] * scale_factor_[1];
    ++*constraint_index;
  }
}

void VehicleModelOptimizerOSQP::AddCarEdgeNoCollision(
    std::vector<std::vector<std::pair<c_int, c_float>>>* const variables, 
    std::vector<c_float>* const lower_bounds, 
    std::vector<c_float>* const upper_bounds,
    int* const constraint_index) {
  const double foh = vehicle_config_.wheel_base() + 
      vehicle_config_.front_overhang_length();

  double s_hat;
  Partial patial;
  std::pair<double, double> bound;

  for (int i = 1; i < num_of_knots_; ++i) {
    // simple model, front corner of car
    s_hat = CalculateCarStationByIndex(i, foh);
    patial = CalculateCarEdgeLaterOffsetCoeff(i, CornerType::FRONT_CORNER);
    bound = CalculateBoundByS(path_boundary_->obstacle_boundary(), s_hat);

    variables->at(num_of_state_ * i).emplace_back(
        *constraint_index, patial.l * scale_factor_[1]);
    variables->at(num_of_state_ * i + 1).emplace_back(
        *constraint_index, patial.theta * scale_factor_[0]);
    lower_bounds->at(*constraint_index) = 
        (bound.first - patial.constant) * scale_factor_[0] * scale_factor_[1];
    upper_bounds->at(*constraint_index) = 
        (bound.second - patial.constant) * scale_factor_[0] * scale_factor_[1];
    ++*constraint_index;
  }
}

} // namespace planning
} // namespace xju