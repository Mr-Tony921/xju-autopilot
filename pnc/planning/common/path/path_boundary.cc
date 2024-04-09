/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/path/path_boundary.h"

#include <cmath>

#include "common/logger/logger.h"
#include "common/math/math_utils.h"

namespace xju {
namespace planning {

PathBoundary::PathBoundary(
    const double start_s, const double delta_s,
    const std::vector<std::pair<double, double>>& lane_boundary,
    const std::vector<std::pair<double, double>>& obstacle_boundary) 
    : start_s_(start_s)
    , delta_s_(delta_s)
    , lane_boundary_(std::move(lane_boundary))
    , obstacle_boundary_(std::move(obstacle_boundary)) {
  MergeBoundary();
}

PathBoundary::PathBoundary(
    const double start_s, const double delta_s,
    const std::vector<std::pair<double, double>>& path_boundary)
    : start_s_(start_s)
    , delta_s_(delta_s)
    , path_boundary_(std::move(path_boundary)) {}

void PathBoundary::set_label(const std::string& label) { label_ = label; }
const std::string& PathBoundary::label() const { return label_; }

const double PathBoundary::start_s() const { return start_s_; }

const double PathBoundary::delta_s() const { return delta_s_; }

void PathBoundary::set_start_s(const double& start_s) { 
  start_s_ = start_s;
}

void PathBoundary::set_delta_s(const double& delta_s) { 
  delta_s_ = delta_s;
}

void PathBoundary::set_lane_boundary(
    const std::vector<std::pair<double, double>>& boundary) {
  lane_boundary_ = boundary;
}

void PathBoundary::set_obstacle_boundary(
    const std::vector<std::pair<double, double>>& boundary) {
  obstacle_boundary_ = boundary;
}

void PathBoundary::set_path_boundary(
    const std::vector<std::pair<double, double>>& boundary) {
  path_boundary_ = boundary;
}

const std::vector<std::pair<double, double>>& 
PathBoundary::lane_boundary() const {
  return lane_boundary_;
}

const std::vector<std::pair<double, double>>& 
PathBoundary::obstacle_boundary() const {
  return obstacle_boundary_;
}

const std::vector<std::pair<double, double>>& 
PathBoundary::path_boundary() const {
  return path_boundary_;
}

void PathBoundary::set_blocking_obstacle_id(const std::string& obs_id) {
  blocking_obstacle_id_ = obs_id;
}

const std::string& PathBoundary::blocking_obstacle_id() const {
  return blocking_obstacle_id_;
}

void PathBoundary::set_is_path_boundary_clear(const bool is_path_boundary_clear) {
  is_path_boundary_clear_ = is_path_boundary_clear;
}

const bool PathBoundary::is_path_boundary_clear() const {
  return is_path_boundary_clear_;
}

void PathBoundary::MergeBoundary() {
  path_boundary_.clear();
  if (lane_boundary_.empty() || obstacle_boundary_.empty()) {
    AERROR << "Lane Boundary or obstacle boundary is empty.";
    return;
  }

  if (lane_boundary_.size() != obstacle_boundary_.size()) {
    AERROR << "Lane Boundary size is not equal to obstacle boundary.";
    return;
  }

  for (int i = 0; i < lane_boundary_.size(); ++i) {
    std::pair<double, double> path_bound;
    path_bound.first = std::fmax(lane_boundary_[i].first, obstacle_boundary_[i].first);
    path_bound.second = std::fmin(lane_boundary_[i].second, obstacle_boundary_[i].second);
    path_boundary_.push_back(path_bound);
  }
}

bool PathBoundary::Vailed() const {
  if (delta_s_ < pnc::kMathEpsilon) {
    AERROR << "Path Boundary is not valiled, delta_s: " << delta_s_ << " < 0.0";
    return false;
  }

  if (path_boundary_.empty()) {
    AERROR << "Path Boundary is not valiled, path boundary is empty.";
    return false;
  }

  return true;
}

double PathBoundary::Length() const {
  if (path_boundary_.empty()) {
    return 0.0;
  }
  return delta_s_ * (path_boundary_.size() - 1);
}

} // namespace planning 
} // namespace xju
