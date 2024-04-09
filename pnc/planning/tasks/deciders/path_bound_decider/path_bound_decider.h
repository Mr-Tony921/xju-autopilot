/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>

#include "planning/common/path/path_boundary.h"
#include "planning/tasks/task.h"

namespace xju {
namespace planning {
struct ObstacleBoundSection {
  size_t start_idx;
  size_t end_idx;
  std::string obstacle_id;
  ObstacleBoundSection() = default;
  ObstacleBoundSection(const size_t input_start_idx, const size_t input_end_idx, const std::string& input_id) {
    start_idx = input_start_idx;
    end_idx = input_end_idx;
    obstacle_id = input_id;
  }
  ObstacleBoundSection(const ObstacleBoundSection& other) {
    start_idx = other.start_idx;
    end_idx = other.end_idx;
    obstacle_id = other.obstacle_id;
  }
};
class PathBoundDecider : public Task {
 public:
  PathBoundDecider(const pnc::TaskConfig& config,
                   const std::shared_ptr<PlanningInternal>& internal);
  void Init(const pnc::TaskConfig& config) override;
  void Reset() override;
  bool Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info, 
               Frame* const frame) override;

 private:
  void InitPathBoundDecider(const Frame& frame,
                            const ReferenceLineInfo& reference_line_info);

  void TrimPathBounds(
      const int path_blocked_idx,
      std::vector<std::tuple<double, double, double>>* const path_boundaries) const;
  void ADCBoundWithBuffer(const ReferenceLineInfo& reference_line_info,
                          double* const adc_bound_left,
                          double* const adc_bound_right);
  bool InitLaneBound(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* const lane_bound);

  bool InitObstacleBound(
      const ReferenceLineInfo& reference_line_info,
      const std::vector<std::tuple<double, double, double>>& lane_bound,
      std::vector<std::tuple<double, double, double>>* const obstacles_bound);

  bool GenerateLaneBound(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* const lane_bound);

  bool GenerateObstaclesBound(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* const obstacles_bound,
      std::vector<ObstacleBoundSection>* const left_obstacle_bound_sections,
      std::vector<ObstacleBoundSection>* const right_obstacle_bound_sections);
  bool UpdateLaneBoundByObstacleBound(
      const std::vector<std::tuple<double, double, double>>& obstacles_bound,
      std::vector<std::tuple<double, double, double>>* const lane_bound);
  bool GetBoundaryFromLanesAndADC(
      const ReferenceLineInfo& reference_line_info,
      std::vector<std::tuple<double, double, double>>* const lane_bound);

  void UpdateObstaclesBoundaryAndCenterLineWithBuffer(
      size_t idx, double left_bound, double right_bound,
      std::vector<std::tuple<double, double, double>>* const path_boundaries,
      double* const center_line);

  bool GeneratePathBoundary(
      const std::vector<std::pair<double, double>>& lane_bound_pair,
      const std::vector<std::pair<double, double>>& obstacles_bound_pair,
      std::vector<std::pair<double, double>>* const path_bound_pair);

  bool UpdatePathBoundWithKappa(
      const size_t idx, const double left_bound, const double right_bound,
      const double width_limited_by_kappa,
      std::vector<std::tuple<double, double, double>>* const lane_bound) const;

  double GetBufferBetweenADCCenterAndEdge();

  std::vector<std::tuple<int, double, double, double, std::string>>
  SortObstaclesForSweepLine(
      const IndexedList<std::string, Obstacle>& indexed_obstacles);

  void DrawDebugInfo(const PathBoundary& path_bound, const std::string& title);
  double GetWidthAccordingtoKappa(const double kappa,
                                  const double margin) const;
  void FiltKappas(const std::vector<double>& origin_kappas,
                  std::vector<double>* const filted_kappas) const;
  double GetWidthAccordingKappa(const double length, const double width,
                                const double kappa, const double margin) const;
  bool ExtendBound(
      const std::vector<ObstacleBoundSection>& left_obstacle_bound_sections,
      const std::vector<ObstacleBoundSection>& right_obstacle_bound_sections,
      std::vector<ObstacleBoundSection>* const externded_left_obstacle_bound_sections,
      std::vector<ObstacleBoundSection>* const externded_right_obstacle_bound_sections,
      std::vector<std::tuple<double, double, double>>* const lane_bound,
      std::vector<std::tuple<double, double, double>>* const obstacles_bound)
      const;
  void ExtendLaneBound(
      const std::vector<ObstacleBoundSection>& sections, const bool is_left,
      const std::vector<std::tuple<double, double, double>>& obstacles_bound,
      std::vector<std::tuple<double, double, double>>* const lane_bound) const;
  bool CheckBlocking(
      const ReferenceLineInfo& reference_line_info,
      const std::vector<ObstacleBoundSection>& externded_left_obstacle_bound_sections,
      const std::vector<ObstacleBoundSection>& externded_right_obstacle_bound_sections,
      std::vector<std::tuple<double, double, double>>* const lane_bound,
      std::vector<std::tuple<double, double, double>>* const obstacles_bound,
      std::string* const blocking_obstacle_id,
      bool* const is_bound_clear) const;
  void CalculateTrajectoryKappas(
      const pnc_map::ReferenceLine& reference_line, const double s_range_start,
      const double s_range_end, const double s_resolution,
      std::vector<double>* const reference_points_kappas);

 private:
  double adc_frenet_s_ = 0.0;
  double adc_frenet_sd_ = 0.0;
  double adc_frenet_l_ = 0.0;
  double adc_frenet_ld_ = 0.0;

  double adc_width_ = 0.0;
  double adc_length_ = 0.0;

  double s_range_start_ = 0.0;
  double s_range_end_ = 0.0;
  double s_resolution_ = 0.0;
  std::vector<double> filted_kappas_;
  bool left_lane_bound_soft_ = true;
  bool right_lane_bound_soft_ = true;
};

}  // namespace planning
}  // namespace xju
