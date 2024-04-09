/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <vector>
#include <string>

namespace xju {
namespace planning {

class PathBoundary {
 public:
  PathBoundary() = default;
  PathBoundary(const double start_s, const double delta_s,
               const std::vector<std::pair<double, double>>& lane_boundary,
               const std::vector<std::pair<double, double>>& obstacle_boundary);
  PathBoundary(const double start_s, const double delta_s,
               const std::vector<std::pair<double, double>>& path_boundary);

  void set_label(const std::string& label);
  const std::string& label() const;

  void set_start_s(const double& start_s);
  const double start_s() const;

  void set_delta_s(const double& delta_s);
  const double delta_s() const;

  void set_lane_boundary(const std::vector<std::pair<double, double>>& boundary);
  const std::vector<std::pair<double, double>>& lane_boundary() const;

  void set_obstacle_boundary(const std::vector<std::pair<double, double>>& boundary);
  const std::vector<std::pair<double, double>>& obstacle_boundary() const;

  void set_path_boundary(const std::vector<std::pair<double, double>>& boundary);
  const std::vector<std::pair<double, double>>& path_boundary() const;

  void set_blocking_obstacle_id(const std::string& obs_id);
  const std::string& blocking_obstacle_id() const;

  void set_is_path_boundary_clear(const bool is_path_boundary_clear);

  const bool is_path_boundary_clear() const;

  bool Vailed() const;

  double Length() const;

 private:
  void MergeBoundary();
 private:
  std::string label_ = "regular";
  double start_s_ = 0.0;
  double delta_s_ = 0.0;
  std::vector<std::pair<double, double>> lane_boundary_;
  std::vector<std::pair<double, double>> obstacle_boundary_;
  // merge lane_boundary_ and obstacle_boundary_
  std::vector<std::pair<double, double>> path_boundary_;
  std::string blocking_obstacle_id_ = "";
  bool is_path_boundary_clear_ = true;
};

} // namespace planning 
} // namespace xju
