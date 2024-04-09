/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "em_lane.pb.h"
#include "vehicle_state.pb.h"
#include "pnc_map/reference_line.h"
#include "common/vehicle_state/vehicle_state_provider.h"
#include <list>
#include <memory>
#include <utility>

namespace xju {
namespace pnc_map {

typedef std::pair<double, double> pose2d_t;

struct TrackingUnit {
  int id;
  uint64_t seq;
  pnc::LocalizePose loc;
  std::vector<pnc::Point3D> fit_input; // x/y: last fit point coord, z: last fit point s
  std::vector<pose2d_t> fit_output;

  TrackingUnit(int id, uint64_t seq, pnc::LocalizePose loc)
      : id(id), seq(seq), loc(loc){};
  void set_fit_input(const std::vector<pnc::Point3D>& input) {
    fit_input = input;
  };
  void set_fit_output(const std::vector<pose2d_t>& output) {
    fit_output = output;
  };
};

class LaneProcess {
 public:
  LaneProcess() = default;
  ~LaneProcess() = default;

  static std::shared_ptr<LaneProcess> GetInstance() {
    static auto instance = std::make_shared<LaneProcess>();
    return instance;
  }

  void UpdateTrackingUnits(const uint64_t seq,
                           const pnc::LocalizePose& em_loc,
                           const pnc::LocalizePose& pnc_loc);

  bool Process(
    const pnc::LaneMark& left_lanemark,
    const pnc::LaneMark& right_lanemark,
    const pnc::LaneInfo& lane_info,
    ReferenceLine* const reference_line);

  bool CenterLineProcess(
    const pnc::LaneInfo& lane_info,
    ReferenceLine* const reference_line);

 private:
  bool ConvertLaneMarkerToPath(
      const pnc::LaneMark& nearest_left_lane,
      const pnc::LaneMark& nearest_right_lane,
      ReferenceLine* const reference_line);

  bool ConvertLaneMarkerPtsToPath(
      const pnc::LaneMark& nearest_left_lane,
      const pnc::LaneMark& nearest_right_lane,
      ReferenceLine* const reference_line);

  bool ConvertSideLaneMarkerToPath(
      const pnc::LaneMark& nearest_left_lane,
      const pnc::LaneMark& nearest_right_lane,
      const pnc::LaneInfo& lane_info,
      ReferenceLine* const reference_line);

  double GetKappa(const double c1, const double c2, 
                  const double c3, const double x);

  double EvaluateCubicPolynomial(
      const double c0, const double c1, 
      const double c2, const double c3, 
      const double x) const;

  bool IsLaneWidthNormal(
      const std::vector<float>& lane_to_lane_width, 
      int* const first_nomal_index);

  bool AddPathPoint(
      const pnc::CubicCurve& lane, 
      const double x, 
      double* const accumulated_s,
      pnc::Path* path,
      pnc::PathPoint* point);

  void ReplaceAbnomalPathPoint(
      const int first_normal_index,
      const pnc::CubicCurve& left_lane,
      const pnc::CubicCurve& right_lane,
      pnc::Path* const path);

  bool ComputePathPointsProfile(const std::vector<pnc::Vec2d>& points,
                                std::vector<double>* const headings,
                                std::vector<double>* const accu_s,
                                std::vector<double>* const kappa,
                                std::vector<double>* const dkappa,
                                std::vector<double>* const ddkappa);

  void NormalizePoints(std::vector<pose2d_t>* xy_points) {
    zero_x_ = xy_points->front().first;
    zero_y_ = xy_points->front().second;
    std::for_each(xy_points->begin(), xy_points->end(),
                  [this](pose2d_t& point) {
                    auto curr_x = point.first;
                    auto curr_y = point.second;
                    pose2d_t xy(curr_x - zero_x_, curr_y - zero_y_);
                    point = std::move(xy);
                  });
  }

  void DeNormalizePoints(std::vector<pose2d_t>* xy_points) {
    std::for_each(xy_points->begin(), xy_points->end(),
                  [this](pose2d_t& point) {
                    auto curr_x = point.first;
                    auto curr_y = point.second;
                    pose2d_t xy(curr_x + zero_x_, curr_y + zero_y_);
                    point = std::move(xy);
                  });
  }

  std::vector<pose2d_t> FitLaneMark(const pnc::LaneMark& lanemark,
                                    const double seg_dist,
                                    const double line_step,
                                    const double keep_dist);

  bool CubicSpline(const std::vector<pnc::Point3D>& input,
                   const double seg_dist,
                   const double line_step,
                   const double curvature_weight,
                   std::vector<pose2d_t>* const output = nullptr,
                   ReferenceLine* const reference_line = nullptr);

  bool PolynomialAndCheck(const std::vector<pnc::Point3D>& input,
                          const double line_step,
                          const double close_dist,
                          const double close_radio,
                          std::vector<pose2d_t>* const output = nullptr,
                          ReferenceLine* const reference_line = nullptr);

  void PathTransform(const pnc::LocalizePose& from,
                     const pnc::LocalizePose& to,
                     std::vector<pose2d_t>* const path);

  void PathTransform(const pnc::LocalizePose& from,
                     const pnc::LocalizePose& to,
                     std::vector<pnc::Point3D>* const path);

 private:
  double zero_x_, zero_y_;
  unsigned int seq_;
  pnc::LocalizePose em_loc_;
  pnc::LocalizePose pnc_loc_;
  std::list<TrackingUnit> tracking_units_;
};

}  // namespace pnc_map
}  // namespace xju
