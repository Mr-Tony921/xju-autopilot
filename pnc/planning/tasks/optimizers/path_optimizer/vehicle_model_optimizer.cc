/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/path_optimizer/vehicle_model_optimizer.h"

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
using xju::pnc::FrenetFramePoint;

void VehicleModelOptimizer::Init() {
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

void VehicleModelOptimizer::set_weights(
    const PathOptimizerWeights& weights) {
  weights_ = weights;
}

void VehicleModelOptimizer::SetCarSpeedInfo(
    const double init_speed, const double target_speed) {
  init_speed_ = init_speed;
  cruise_speed_ = target_speed;
  dkappa_limit_ = 2.0 * dkappa_limit_ / (init_speed_ + cruise_speed_ + kMathEpsilon);
}

void VehicleModelOptimizer::SetTarget(const double x,
                                    const double y,
                                    const double theta) {
  has_target_ = true;
  target_x_ = x;
  target_y_ = y;
  target_theta_ = theta;
}

void VehicleModelOptimizer::CalculateTargetInfo(
    const pnc_map::ReferenceLine& reference_line) {
  pnc::SLPoint target_sl;
  reference_line.XYToSL(pnc::Vec2d(target_x_, target_y_), &target_sl);
  double target_s = target_sl.s();
  target_l_ = target_sl.l();
  if (std::abs(target_l_) < 0.01) {
    target_l_ = 0.0;
  }
  for (auto i = 0; i < reference_points_.size(); ++i) {
    if (target_s <= reference_points_[i].s()) {
      target_index_ = i;
      break;
    }
  }
  target_index_ = std::max(1, target_index_ - 1);
}

void VehicleModelOptimizer::ResetTargetInfo() {
  has_target_ = false;
  target_x_ = 0.0;
  target_y_ = 0.0;
  target_theta_ = 0.0;
  target_l_ = 0.0;
  target_theta_error_ = 0.0;
  target_index_ = std::numeric_limits<int>::max();
}

bool VehicleModelOptimizer::OptimizePath(
    const ReferenceLine& reference_line,
    const PathPoint& start_point,
    const PathBoundary* const boundary) {
  // 1. Init local virables
  InitLocalVirables(start_point, boundary);

  // 2. calculate reference points
  CalculateReferencePoints(reference_line);

  // 3. check
  if (!Check()) {
    AERROR << "Input of VehicleModelOptimizer ERROR.";
    ResetTargetInfo();
    return false;
  }

  // 4. calculate start point constraint
  CalculateStartPointConstraint(start_point);

  // 5. calculate target info
  if (has_target_) {
    CalculateTargetInfo(reference_line);
  }

  // 6. construct optimizal problem and solve
  if (!Optimize()) {
    AERROR << " Optimize of VehicleModelOptimizer ERROR With " << boundary_label_ << " Boundary.";
    ResetTargetInfo();
    return false;
  }

  ResetTargetInfo();
  status_ = true;
  return true;
}

bool VehicleModelOptimizer::GetOptimizePath(
    PathData* const path_data) {
  frenet_points_.clear();
  frenet_points_.resize(num_of_knots_);
  FrenetFramePoint pt{};
  for (int i = 0; i < num_of_knots_; ++i) {
    pt.set_s(reference_points_[i].s());
    pt.set_l(solution_[i].x[0]);
    pt.set_heading_error(solution_[i].x[1]);
    pt.set_kappa(solution_[i].x[2]);

    if (i == 0) {
      pt.set_dkappa(planning_start_point_.dkappa());
    } else {
      pt.set_dkappa(solution_[i - 1].u[0]);
    }
    frenet_points_[i] = pt;
  }
  path_data->set_reference_points(reference_points_);
  return path_data->set_frenet_points(frenet_points_);
}

void VehicleModelOptimizer::CalculateReferencePoints(
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
}

void VehicleModelOptimizer::CalculateStartPointConstraint(
  const PathPoint& start_point) {
  const PathPoint& reference_point = reference_points_.front();
  double l = pnc::Distance(start_point, reference_point);
  const double dx = start_point.x() - reference_point.x();
  const double dy = start_point.y() - reference_point.y();
  const double cross = dy * std::cos(reference_point.theta()) - 
                       dx * std::sin(reference_point.theta());
  l = (cross > 0.0 ? 1.0 : -1.0) * l;
  double theta_error = 
      pnc::NormalizeAngle(start_point.theta() - reference_point.theta());

  init_state_ =
      (Eigen::VectorXd(3) << l, theta_error, start_point.kappa())
          .finished();
  ADEBUG << "Start Point: " << start_point.ShortDebugString();
  ADEBUG << "First Reference point: " << reference_point.ShortDebugString();
  ADEBUG << "init state: " << init_state_;
}

void VehicleModelOptimizer::InitLocalVirables(
    const PathPoint& start_point,
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
  ADEBUG << "boundary_start_s_: " << boundary_start_s_;
  ADEBUG << "boundary_delta_s_: " << boundary_delta_s_;
  ADEBUG << "boundary_end_s_: " << boundary_end_s_;
  ADEBUG << "planning_end_s_: " << planning_end_s_;
}

bool VehicleModelOptimizer::Check() {
  if (!path_boundary_->Vailed()) {
    AERROR << path_boundary_->label() << " path boundary is not vailed!";
    return false;
  }
  if (boundary_start_s_ > boundary_end_s_ - kMathEpsilon) {
    AERROR << "Start_s >= End_s of boundary";
    return false;
  }
  if (reference_points_.size() < 2) {
    AERROR << "Reference Points size < 2";
    return false;
  }
  return true;
}

bool VehicleModelOptimizer::Optimize() {
  // init size of OcpQp problem
  ocp_qps_.clear();
  ocp_qps_.resize(num_of_knots_);
  SetWarmStartValue();

  // dynamics
  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    double kappa = reference_points_[i].kappa();
    double delta_s = index_delta_s_map_.find(i + 1)->second;
    CalculateVehicleStateSpace(kappa, delta_s, &sys_, &control_, &dis_);
    ocp_qps_[i].A = sys_;
    ocp_qps_[i].B = control_;
    ocp_qps_[i].b = dis_;
  }

  // cost function
  const double speed = (cruise_speed_ + init_speed_) / 2.0;
  Q_(0, 0) = weights_.l();
  Q_(1, 1) = weights_.theta() * std::fmax(speed * speed, 5.0);
  Q_(2, 2) = weights_.kappa();
  R_(0, 0) = weights_.dkappa();
  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    ocp_qps_[i].Q = Q_;
    ocp_qps_[i].R = R_;
    ocp_qps_[i].S = S_;
    // ocp_qps_[i].q = q_;
    ocp_qps_[i].r = r_;
    q_[2] = -weights_.kappa() * reference_points_[i].kappa();
    ocp_qps_[i].q = q_;
  }
  q_[2] = -weights_.kappa() * reference_points_[num_of_knots_ - 1].kappa();
  ocp_qps_[num_of_knots_ - 1].Q = Q_;
  ocp_qps_[num_of_knots_ - 1].q = q_;

  // box constraints
  ocp_qps_[0].idxbx = {0, 1, 2};
  ocp_qps_[0].lbx = init_state_;
  ocp_qps_[0].ubx = init_state_;
  for (int i = 1; i < num_of_knots_; ++i) {
    std::pair<double, double> bound = 
        CalculateBoundByS(path_boundary_->path_boundary(), reference_points_[i].s());
    ocp_qps_[i].idxbx = {0, 1, 2};
    ocp_qps_[i].lbx = 
        (Eigen::VectorXd(3) << bound.first, 
                               -FLAGS_heading_error_bound,
                               -kappa_limit_).finished();
    ocp_qps_[i].ubx = 
        (Eigen::VectorXd(3) << bound.second, 
                               FLAGS_heading_error_bound,
                               kappa_limit_).finished();
  }

  // Zl_(0, 0) = weights_.target_l();
  // Zl_(1, 1) = weights_.target_theta();
  // Zu_(0, 0) = weights_.target_l();
  // Zu_(1, 1) = weights_.target_theta();
  // lls_ = (Eigen::VectorXd(2) << weights_.max_target_l_slack(),
  //         weights_.max_target_theta_slack())
  //            .finished();
  // lus_ = (Eigen::VectorXd(2) << weights_.max_target_l_slack(),
  //         weights_.max_target_theta_slack())
  //            .finished();
  for (auto i = target_index_; i < num_of_knots_; ++i) {
    // ocp_qps_[i].Zl = Zl_;
    // ocp_qps_[i].Zu = Zu_;
    // ocp_qps_[i].zl = zl_;
    // ocp_qps_[i].zu = zu_;
    // ocp_qps_[i].idxs = {0, 1};
    // ocp_qps_[i].lls = lls_;
    // ocp_qps_[i].lus = lus_;

    ocp_qps_[i].idxbx = {0, 1, 2};
    ocp_qps_[i].lbx =
        (Eigen::VectorXd(3) << -target_l_, -target_theta_error_, -kappa_limit_)
            .finished();
    ocp_qps_[i].ubx =
        (Eigen::VectorXd(3) << target_l_, target_theta_error_, kappa_limit_)
            .finished();
  }

  // dkappa limit is error !!!
  // negative_kl = (Eigen::VectorXd(1) << -dkappa_limit_).finished();
  // positive_kl = (Eigen::VectorXd(1) << dkappa_limit_).finished();
  // for (int i = 0; i < num_of_knots_ - 1; ++i) {
  //   double delta_s = index_delta_s_map_.find(i + 1)->second;
  //   ocp_qps_[i].idxbu = {0};
  //   ocp_qps_[i].lbu = negative_kl * delta_s;
  //   ocp_qps_[i].ubu = positive_kl * delta_s;
  // }
  // general constraints
  // front corner and front alex of car
  const double wheel_base = vehicle_config_.wheel_base();
  const double foh = vehicle_config_.wheel_base() + 
      vehicle_config_.front_overhang_length();
  const double roh = -vehicle_config_.back_overhang_length();
  
  if (path_boundary_->is_path_boundary_clear()) {
    for (int i = 1; i < num_of_knots_; ++i) {
      // tire position constraints, in lane and no collision
      double s_hat1 = CalculateCarStationByIndex(i, wheel_base);
      auto patial1 = CalculateCarEdgeLaterOffsetCoeff(i, CornerType::FRONT_AXLE);
      auto bound1 = CalculateBoundByS(path_boundary_->lane_boundary(), s_hat1);

      C_lane_ << patial1.l, patial1.theta, 0.0;
      lg_lane_ << bound1.first - patial1.constant;
      ug_lane_ << bound1.second - patial1.constant;
      ocp_qps_[i].C = C_lane_;
      ocp_qps_[i].D = D_lane_;
      ocp_qps_[i].lg = lg_lane_;
      ocp_qps_[i].ug = ug_lane_;
    }
  } else {
    for (int i = 1; i < num_of_knots_; ++i) {
      // tire position constraints, in lane and no collision
      double s_hat1 = CalculateCarStationByIndex(i, wheel_base);
      auto patial1 = CalculateCarEdgeLaterOffsetCoeff(i, CornerType::FRONT_AXLE);
      auto bound1 = CalculateBoundByS(path_boundary_->path_boundary(), s_hat1);
    
      // car corner position constraints, no collision
      double s_hat2 = CalculateCarStationByIndex(i, foh);
      auto patial2 = CalculateCarEdgeLaterOffsetCoeff(i, CornerType::FRONT_CORNER);
      auto bound2 = CalculateBoundByS(path_boundary_->obstacle_boundary(), s_hat2);

      double s_hat3 = CalculateCarStationByIndex(i, roh);
      auto patial3 = CalculateCarEdgeLaterOffsetCoeff(i, CornerType::REAR_CORNER);
      auto bound3 = CalculateBoundByS(path_boundary_->obstacle_boundary(), s_hat3);

      C_ << patial1.l, patial1.theta, 0.0,
                    patial2.l, patial2.theta, 0.0,
                    patial3.l, patial3.theta, 0.0;
      lg_ << bound1.first - patial1.constant,
                     bound2.first - patial2.constant,
                     bound3.first - patial3.constant;
      ug_ << bound1.second - patial1.constant,
                     bound2.second - patial2.constant,
                     bound3.second - patial3.constant;
      ocp_qps_[i].C = C_;
      ocp_qps_[i].D = D_;
      ocp_qps_[i].lg = lg_;
      ocp_qps_[i].ug = ug_;
    }
  }
  
  pnc::hpipm::OcpQpIpmSolverSettings solver_settings;
  SetIpmSolverSettings(&solver_settings);

  pnc::hpipm::OcpQpIpmSolver solver;
  solver.set_solver_settings(solver_settings);
  const auto status = solver.solve(ocp_qps_, solution_);
  cost_ = std::fabs(solver.solver_statistics().obj_val);
  return status == pnc::hpipm::HpipmStatus::Success;
}

void VehicleModelOptimizer::SetIpmSolverSettings(
    pnc::hpipm::OcpQpIpmSolverSettings* const solver_settings) {
  solver_settings->mode = pnc::hpipm::HpipmMode::Balance;
  solver_settings->iter_max = 50;
  solver_settings->alpha_min = 1e-8;
  solver_settings->mu0 = 1e2;
  solver_settings->tol_stat = 1e-05;
  solver_settings->tol_eq = 1e-05;
  solver_settings->tol_ineq = 1e-05;
  solver_settings->tol_comp = 1e-05;
  solver_settings->reg_prim = 1e-12;
  solver_settings->warm_start = 1;
  solver_settings->pred_corr = 1;
  solver_settings->ric_alg = 0;
  solver_settings->split_step = 1;
}

void VehicleModelOptimizer::CalculateVehicleStateSpace(
    const double kappa,
    const double delta_s,
    Eigen::MatrixXd* const sys,
    Eigen::MatrixXd* const control,
    Eigen::VectorXd* const dis) {
  const double rear_axle_to_hitch =
      vehicle_config_.rear_axle_to_hitch();

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

std::pair<double, double> 
VehicleModelOptimizer::CalculateBoundByS(
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
  bound.first = std::fmax(bounds[front_index].first, bounds[back_index].first) 
                    + half_width_;
  bound.second = std::fmin(bounds[front_index].second, bounds[back_index].second)
                     - half_width_;
  return bound;
}

double VehicleModelOptimizer::CalculateCarStationByIndex(
    const size_t index, const double length) {
  const auto& reference_point = reference_points_[index];
  return reference_point.s() + length;
}

VehicleModelOptimizer::Partial 
VehicleModelOptimizer::CalculateCarEdgeLaterOffsetCoeff(
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

VehicleModelOptimizer::Partial 
VehicleModelOptimizer::CalculateCarEdgeLaterOffsetCoeffTest(
    const CornerType& type, const double kappa) {
  // const double kappa = reference_points_[index].kappa();
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

void VehicleModelOptimizer::GetCarCornerParams(
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

void VehicleModelOptimizer::SetWarmStartValue() {
  solution_.resize(num_of_knots_);
  int num_of_state = 3;

  for (int i = 0; i < num_of_knots_; ++i) {
    if (solution_[i].x.size() != num_of_state) {
      solution_[i].x = Eigen::VectorXd::Zero(num_of_state);
    }
    if (solution_[i].u.size() != 1) {
      solution_[i].u = Eigen::VectorXd::Zero(1);
    }
  }
}

} // namespace planning
} // namespace xju