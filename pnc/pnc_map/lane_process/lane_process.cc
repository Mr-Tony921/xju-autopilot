/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "lane_process.h"
#include "pnc_map/pnc_map_gflags/pnc_map_gflags.h"

#include "common/logger/logger.h"
#include "common/util/coordinate.h"
#include "common/math/vec2d.h"
#include "common/math/curve_math.h"
#include "common/math/interpolation_1d.h"
#include "common/math/smoothness/fem_pos_deviation_osqp_interface.h"
#include "common/math/smoothness/cubic_spline_smoother.h"
#include <algorithm>
#include <limits>
#include <fstream>

namespace xju {
namespace pnc_map {

void LaneProcess::UpdateTrackingUnits(const uint64_t seq,
                                      const pnc::LocalizePose& em_loc,
                                      const pnc::LocalizePose& pnc_loc) {
  seq_ = seq;
  em_loc_ = em_loc;
  pnc_loc_ = pnc_loc;
  for (auto it = tracking_units_.begin(); it != tracking_units_.end();) {
    if (it->seq != seq_ && it->seq != seq_ - 1) {
      it = tracking_units_.erase(it);
    } else {
      ++it;
    }
  }
}

bool LaneProcess::Process(
    const pnc::LaneMark& left_lanemark,
    const pnc::LaneMark& right_lanemark,
    const pnc::LaneInfo& lane_info,
    ReferenceLine* const refenrence_line) {
  // ADEBUG << "LaneProcess : start!";

  if (FLAGS_use_center_line) {
    return CenterLineProcess(lane_info, refenrence_line);
  }

  if (FLAGS_use_line_pts) {
    return ConvertLaneMarkerPtsToPath(left_lanemark, right_lanemark,
                                      refenrence_line);
  }

  if (!left_lanemark.has_curve_vehicle_coord() || 
      !right_lanemark.has_curve_vehicle_coord()) {
    AERROR << "LaneProcess : curve param is missing!";
    return false;
  }

  bool is_quality_high = !((lane_info.leftline_quality() == 2 || 
                            lane_info.leftline_quality() == 3) ^
                           (lane_info.rightline_quality() == 2 || 
                            lane_info.rightline_quality() == 3));

  if (!is_quality_high && FLAGS_use_side_lane) {
    if (!ConvertSideLaneMarkerToPath(
             left_lanemark, right_lanemark, lane_info, refenrence_line)) {
      AERROR << "LaneProcess : lane generation on side failed!";
      return false;
    }
  } else if (!ConvertLaneMarkerToPath(
        left_lanemark, right_lanemark, refenrence_line)) {
    AERROR << "LaneProcess : lane generation failed!";
    return false;
  }
  return true;
}

bool LaneProcess::CenterLineProcess(const pnc::LaneInfo& lane_info,
                                    ReferenceLine* const reference_line) {
  // generate original points
  std::vector<pnc::Point3D> ori_pts;
  if (!FLAGS_use_line_pts) {
    if (!lane_info.has_curve_center()) {
      AERROR << "LaneProcess : curve param is missing!";
      return false;
    }

    const auto& curve = lane_info.curve_center();
    for (double x = curve.start(); x <= curve.end(); x += 1.0) {
      pnc::Point3D p{};
      p.set_x(x);
      p.set_y(EvaluateCubicPolynomial(curve.c0(), curve.c1(), curve.c2(),
                                      curve.c3(), x));
      ori_pts.emplace_back(std::move(p));
    }
  } else {
    if (!lane_info.pts_center().size()) {
      AERROR << "LaneProcess : pts param is missing!";
      return false;
    }

    ori_pts = std::vector<pnc::Point3D>(lane_info.pts_center().begin(),
                                        lane_info.pts_center().end());
  }

  if (ori_pts.empty()) {
    AERROR << "LaneProcess : ori pts is empty!";
    return false;
  }

  // coordinates transformation
  if (FLAGS_enable_reference_line_pose_correction) {
    PathTransform(em_loc_, pnc_loc_, &ori_pts);
  }

  // get tracking infomation from last seq
  std::vector<pnc::Point3D> last_input;
  for (auto const& unit : tracking_units_) {
    if (FLAGS_enable_lane_mark_tracking && seq_ - 1 == unit.seq &&
        lane_info.lane_id() == unit.id) {
      last_input = unit.fit_input;
      PathTransform(unit.loc, pnc_loc_, &last_input);
      break;
    }
  }

  // generate fit_data
  static constexpr double tracking_dist = 100.0;
  std::vector<pnc::Point3D> fit_data;
  if (last_input.empty()) {
    fit_data = ori_pts;
  } else {
    auto fb = last_input.begin();
    auto fe = last_input.end();
    double min_d2 = std::numeric_limits<double>::max();
    bool find_fb = false;
    auto target = ori_pts[0];
    for (auto it = last_input.begin(); it != last_input.end(); ++it) {
      double tmp_d2 = (it->x() - target.x()) * (it->x() - target.x()) +
                      (it->y() - target.y()) * (it->y() - target.y());
      if (tmp_d2 <= min_d2) {
        min_d2 = tmp_d2;
        continue;
      }
      if (!find_fb) {
        fb = std::prev(it);
        find_fb = true;
        continue;
      }
      if (it->z() - fb->z() > tracking_dist) {
        fe = it;
        break;
      }
    }
    if (min_d2 < 4.0) {
      fit_data = std::vector<pnc::Point3D>(fb, fe);
    }
    int fb_pos = 0;
    if (!fit_data.empty()) {
      min_d2 = std::numeric_limits<double>::max();
      target = fit_data.back();
      for (auto it = ori_pts.begin(); it != ori_pts.end(); ++it, ++fb_pos) {
        double tmp_d2 = (it->x() - target.x()) * (it->x() - target.x()) +
                        (it->y() - target.y()) * (it->y() - target.y());
        if (tmp_d2 > min_d2) {
          break;
        }
        min_d2 = tmp_d2;
      }
    }
    fit_data.insert(fit_data.end(), ori_pts.begin() + fb_pos, ori_pts.end());
  }
  double accu_s = 0.0;
  for (auto i = 0; i < fit_data.size(); ++i) {
    if (i > 0) {
      accu_s += std::hypot(fit_data[i].x() - fit_data[i - 1].x(),
                           fit_data[i].y() - fit_data[i - 1].y());
    }
    fit_data[i].set_z(accu_s);
  }

  // check if polynomial fitting can be used
  static constexpr double line_step = 0.5;
  static constexpr double close_dist = 0.2;
  static constexpr double close_radio = 0.9;
  if (PolynomialAndCheck(fit_data, line_step, close_dist, close_radio, nullptr,
                         reference_line)) {
    TrackingUnit unit(lane_info.lane_id(), seq_, pnc_loc_);
    unit.set_fit_input(std::move(fit_data));
    tracking_units_.emplace_back(std::move(unit));
    return true;
  }

  // cubic spline fitting
  static constexpr double seg_dist = 5.0;
  static constexpr double smooth = 0.01;
  if (CubicSpline(fit_data, seg_dist, line_step, smooth, nullptr,
                  reference_line)) {
    TrackingUnit unit(lane_info.lane_id(), seq_, pnc_loc_);
    unit.set_fit_input(std::move(fit_data));
    tracking_units_.emplace_back(std::move(unit));
    return true;
  }

  return false;
}

bool LaneProcess::ConvertLaneMarkerToPath(
    const pnc::LaneMark& left_lanemark,
    const pnc::LaneMark& right_lanemark,
    ReferenceLine* const refenrence_line) {
  const auto& left_lane = left_lanemark.curve_vehicle_coord();
  const auto& right_lane = right_lanemark.curve_vehicle_coord();

  float lane_start = std::fmax(left_lane.start(), right_lane.start());
  float lane_end = std::fmin(left_lane.end(), right_lane.end());
  float lane_delta_x = lane_end - lane_start;
  if (lane_delta_x <= 50) {
    AERROR << "LaneProcess : lane distance is too short! Dist: " << lane_delta_x;
    // return false;
  }
  if (lane_start >= lane_end) {
    AERROR << "LaneProcess : start pos is further than end pos! Pos_s : " 
           << lane_start
           << ", Pose_e : " << lane_end;
    return false;
  }
  if (lane_start > 20) {
    AERROR << "LaneProcess : start pos is far away from vehicle! Pos_start : " 
           << lane_start;
  }

  // path generation
  pnc::Path path;
  pnc::CubicCurve ref_curve;

  // path function: y = c3*x^3 + c2*x^2 + c1*x + c0
  double path_c0 = (left_lane.c0() + right_lane.c0()) * 0.5;
  double path_c1 = (left_lane.c1() + right_lane.c1()) * 0.5;
  double path_c2 = (left_lane.c2() + right_lane.c2()) * 0.5;
  double path_c3 = (left_lane.c3() + right_lane.c3()) * 0.5;

  ref_curve.set_c0(path_c0);
  ref_curve.set_c1(path_c1);
  ref_curve.set_c2(path_c2);
  ref_curve.set_c3(path_c3);
  ref_curve.set_start(lane_start);
  ref_curve.set_end(lane_end);
  double accumulated_s = 0;
  std::vector<float> lane_to_lane_width;

  for (float x = lane_start; x < lane_end; x += 0.5) {
    auto* point = path.add_path_point();
    if (!AddPathPoint(ref_curve, x, &accumulated_s, &path, point)) {
      break;
    }

    if (FLAGS_use_side_lane_in_lane_change) {
      float left_lane_y =
          EvaluateCubicPolynomial(
              left_lane.c0(), left_lane.c1(), left_lane.c2(), left_lane.c3(), x);
      float right_lane_y =
          EvaluateCubicPolynomial(
              right_lane.c0(), right_lane.c1(), right_lane.c2(), right_lane.c3(), x);
      lane_to_lane_width.push_back(left_lane_y - right_lane_y);
    }
  }

  std::vector<pnc::PathPoint> path_points(
      path.mutable_path_point()->begin(), path.mutable_path_point()->end());
  refenrence_line->set_path_points(std::move(path_points));

  return true;
}

bool LaneProcess::ConvertLaneMarkerPtsToPath(
    const pnc::LaneMark& left_lanemark,
    const pnc::LaneMark& right_lanemark,
    ReferenceLine* const refenrence_line) {
  // param
  static constexpr double segment_dist = 5.0;
  static constexpr double line_step = 0.5;
  static constexpr double tracking_dist = 100.0;

  // generate left/right line
  auto left_points = FitLaneMark(left_lanemark, segment_dist, line_step, tracking_dist);
  auto right_points = FitLaneMark(right_lanemark, segment_dist, line_step, tracking_dist);
  auto pts_size = std::min(left_points.size(), right_points.size());
  if (pts_size == 0) {
    AERROR << "LaneProcess : Lane mark fitting failed!";
    return false;
  }

  // generate middle line
  std::vector<pose2d_t> fit_points(pts_size);
  for (auto i = 0; i < fit_points.size(); ++i) {
    fit_points[i].first = 0.5 * (left_points[i].first + right_points[i].first);
    fit_points[i].second = 0.5 * (left_points[i].second + right_points[i].second);
  }

  // deviation smoother
  if (FLAGS_enable_smooth_reference_line) {
    NormalizePoints(&fit_points);
    const double box_ratio = 1.0 / std::sqrt(2.0);
    std::vector<double> bounds(fit_points.size(),
                               0.5 * box_ratio);  // lateralbound
    bounds.front() = 0.0;
    bounds.back() = 0.0;

    xju::pnc::FemPosDeviationOsqpInterface solver;
    solver.set_weight_fem_pos_deviation(
        1e7);                              // config_.weight_fem_pos_deviation()
    solver.set_weight_path_length(0.0);    // config_.weight_path_length()
    solver.set_weight_ref_deviation(1e3);  // config_.weight_ref_deviation()
    solver.set_max_iter(500);              // config_.max_iter()
    solver.set_time_limit(0.0);            // config_.time_limit()
    solver.set_verbose(false);             // config_.verbose()
    solver.set_scaled_termination(true);   // config_.scaled_termination()
    solver.set_warm_start(true);           // config_.warm_start()
    solver.set_ref_points(fit_points);
    solver.set_bounds_around_refs(bounds);

    if (!solver.Solve()) {
      AERROR << "LaneProcess : smoother filed!";
      return false;
    }

    const auto& opt_x = solver.opt_x();
    const auto& opt_y = solver.opt_y();
    if (opt_x.size() < 2 || opt_y.size() < 2 || opt_x.size() != opt_y.size()) {
      AERROR << "LaneProcess : smoother output error!";
      return false;
    }
    fit_points.resize(opt_x.size());
    for (auto i = 0; i < opt_x.size(); ++i) {
      fit_points[i].first = opt_x[i];
      fit_points[i].second = opt_y[i];
    }
    DeNormalizePoints(&fit_points);
  }

  // coordinates transformation
  if (FLAGS_enable_reference_line_pose_correction) {
    PathTransform(em_loc_, pnc_loc_, &fit_points);
  }

  // path generation
  std::vector<pnc::Vec2d> new_points(fit_points.size());
  for (auto i = 0; i < fit_points.size(); ++i) {
    new_points[i] = pnc::Vec2d(fit_points[i].first, fit_points[i].second);
  }
  std::vector<double> headings, accu_s, kappa, dkappa, ddkappa;
  if (!ComputePathPointsProfile(new_points, &headings, &accu_s, &kappa, &dkappa,
                                &ddkappa)) {
    AERROR << "LaneProcess : ComputePathPointProfile failed";
    return false;
  }
  std::vector<pnc::PathPoint> path_points;
  for (auto i = 0; i < new_points.size(); ++i) {
    pnc::PathPoint path_point;
    path_point.set_x(new_points[i].x());
    path_point.set_y(new_points[i].y());
    path_point.set_z(0.0);
    path_point.set_theta(headings[i]);
    path_point.set_kappa(kappa[i]);
    path_point.set_dkappa(dkappa[i]);
    path_point.set_ddkappa(ddkappa[i]);
    path_point.set_s(accu_s[i]);
    path_points.emplace_back(std::move(path_point));
  }
  refenrence_line->set_path_points(std::move(path_points));

  return true;
}

// 靠近原点的n个点宽度小于：double offset_width = FLAGS_lane_width_in_process_lane_change;
// 只处理靠近原点的异常值，即如果是车道消亡，则不会被处理
bool LaneProcess::IsLaneWidthNormal(
    const std::vector<float>& lane_to_lane_width, int* const first_nomal_index) {
  float offset_width = FLAGS_lane_width_in_process_lane_change;
  int n = 2;
  int count = 0;
  for (size_t i = 0; i < lane_to_lane_width.size(); i++) {
    if (lane_to_lane_width[i] < offset_width) {
      count++;
    } else {
      if (count >= n) {
        *first_nomal_index = i;
        return false;
      } else {
        return true;
      }
    }
  }
  return true;
}

// 1 use first normal point position to get displace side lane
// the displace side lane should contain the first normal point
// y = c3 *x^3 + c2*x^2 + c1*x + (c0 + offset) 代入第一个正常点求解平移量diff
// 2 recaculate abnormal path points
// 3 TODO: if the junction point should be considered
void LaneProcess::ReplaceAbnomalPathPoint(
    const int first_normal_index,
    const pnc::CubicCurve& left_lane,
    const pnc::CubicCurve& right_lane,
    pnc::Path* const path) {
  // determine which lane will be used to get path
  // 取第一个正常点x，y绝对值更小的那条为目标参考线
  pnc::CubicCurve ref_lane;
  float x = path->path_point(first_normal_index).x();
  float y = path->path_point(first_normal_index).y();

  float left_offset = y - left_lane.c0() - left_lane.c1() * x - 
      left_lane.c2() * x * x - left_lane.c3() * x * x * x;
  float right_offset = y - right_lane.c0() - right_lane.c1() * x - 
      right_lane.c2() * x * x - right_lane.c3() * x * x * x;

  float original_path_y = EvaluateCubicPolynomial(
    left_lane.c0() + right_lane.c0(),
    left_lane.c1() + right_lane.c1(),
    left_lane.c2() + right_lane.c2(),
    left_lane.c3() + right_lane.c3(),
    0);

  if (original_path_y < 0) {
    ref_lane = right_lane;
    ref_lane.set_c0(right_lane.c0() + right_offset);
  } else {
    ref_lane = left_lane;
    ref_lane.set_c0(left_lane.c0() + left_offset);
  }

  // repalce abnormal path points
  double accumulated_s = 0;
  for (int i = 0; i < first_normal_index; i++) {
    auto *point = path->mutable_path_point(i);
    AddPathPoint(ref_lane, point->x(), &accumulated_s, path, point);
  }
}

bool LaneProcess::ConvertSideLaneMarkerToPath(
    const pnc::LaneMark& left_lanemark,
    const pnc::LaneMark& right_lanemark,
    const pnc::LaneInfo& lane_info,
    ReferenceLine* const refenrence_line) {
  const auto& side_lane = (lane_info.leftline_quality() == 2 || 
                           lane_info.leftline_quality() == 3) 
                          ? left_lanemark.curve_vehicle_coord()
                            : right_lanemark.curve_vehicle_coord();
  const auto& is_left_lane = (lane_info.leftline_quality() == 2 || 
                              lane_info.leftline_quality() == 3) 
                             ? true : false;

  const auto& left_lane = left_lanemark.curve_vehicle_coord();
  const auto& right_lane = right_lanemark.curve_vehicle_coord();

  float lane_start = side_lane.start();
  float lane_end = side_lane.end();
  float lane_delta_x = lane_end - lane_start;
  if (lane_delta_x <= 50) {
    AERROR << "LaneProcess : lane distance is too short! Dist: " << lane_delta_x;
    // return false;
  }
  if (lane_start >= lane_end) {
    AERROR << "LaneProcess : start pos is further than end pos! Pos_s : "
           << lane_start << ", Pose_e : " << lane_end;
    return false;
  }
  if (lane_start > 20) {
    AERROR << "LaneProcess : start pos is far away from vehicle! Pos_start : " 
           << lane_start;
  }

  // path generation
  pnc::Path path;
  pnc::CubicCurve ref_curve;

  // double offset_width = 1.3;
  double offset_width =
      pnc::Clamp((left_lane.c0() - right_lane.c0()) * 0.5, 1.5, FLAGS_side_lane_offset_width);

  double path_c0 = (is_left_lane) ? side_lane.c0() - offset_width : side_lane.c0() + offset_width;

  double path_c1 = side_lane.c1();
  double path_c2 = side_lane.c2();
  double path_c3 = side_lane.c3();

  ref_curve.set_c0(path_c0);
  ref_curve.set_c1(path_c1);
  ref_curve.set_c2(path_c2);
  ref_curve.set_c3(path_c3);
  ref_curve.set_start(lane_start);
  ref_curve.set_end(lane_end);
  double accumulated_s = 0;
  for (float x = lane_start; x < lane_end; x += 1.0) {
    auto *point = path.add_path_point();
    if (!AddPathPoint(ref_curve, x, &accumulated_s, &path, point)) {
      break;
    }
  }

  std::vector<pnc::PathPoint> path_points(
      path.mutable_path_point()->begin(), path.mutable_path_point()->end());
  refenrence_line->set_path_points(std::move(path_points));

  return true;
}

// add point until accumulated_s is enough
bool LaneProcess::AddPathPoint(
    const pnc::CubicCurve& lane,
    const double x,
    double* const accumulated_s,
    pnc::Path* const path,
    pnc::PathPoint* const point) {
  // auto *point = path->add_path_point();
  float y = EvaluateCubicPolynomial(lane.c0(), lane.c1(), lane.c2(), lane.c3(), x);
  point->set_x(x);
  point->set_y(y);

  // if accumulated s is meaningful for path?
  if (path->path_point_size() > 1) {
    auto &pre_point = path->path_point(path->path_point_size() - 2);
    *accumulated_s += std::hypot(x - pre_point.x(), y - pre_point.y());
  }
  point->set_s(*accumulated_s);
  double theta = std::atan2(3 * lane.c3() * x * x + 2 * lane.c2() * x + lane.c1(), 1);
  point->set_theta(theta);
  point->set_kappa(GetKappa(lane.c1(), lane.c2(), lane.c3(), x));
  const double k1 = GetKappa(lane.c1(), lane.c2(), lane.c3(), x - 0.0001);
  const double k2 = GetKappa(lane.c1(), lane.c2(), lane.c3(), x + 0.0001);
  const double dkappa = (k2 - k1) / 0.0002;
  point->set_dkappa(dkappa);
  if (*accumulated_s > FLAGS_max_navigation_length) {
    return false;
  }

  return true;
}

double LaneProcess::EvaluateCubicPolynomial(
    const double c0, const double c1,
    const double c2, const double c3,
    const double x) const {
  return ((c3 * x + c2) * x + c1) * x + c0;
}

double LaneProcess::GetKappa(
    const double c1, const double c2, const double c3, const double x) {
  const double dy = 3 * c3 * x * x + 2 * c2 * x + c1;
  const double d2y = 6 * c3 * x + 2 * c2;
  return d2y / std::pow((1 + dy * dy), 1.5);
}

bool LaneProcess::ComputePathPointsProfile(
    const std::vector<pnc::Vec2d>& points,
    std::vector<double>* const headings,
    std::vector<double>* const accu_s,
    std::vector<double>* const kappa,
    std::vector<double>* const dkappa,
    std::vector<double>* const ddkappa) {
  ACHECK(headings);
  ACHECK(accu_s);
  ACHECK(kappa);
  ACHECK(dkappa);
  headings->clear();
  accu_s->clear();
  kappa->clear();
  dkappa->clear();
  ddkappa->clear();

  std::size_t size = points.size();
  if (size < 2) {
    return false;
  }

  std::vector<double> dxs;
  std::vector<double> dys;
  std::vector<double> y_over_s_first_derivatives;
  std::vector<double> x_over_s_first_derivatives;
  std::vector<double> y_over_s_second_derivatives;
  std::vector<double> x_over_s_second_derivatives;
  std::vector<double> y_over_s_third_derivatives;
  std::vector<double> x_over_s_third_derivatives;

  // dxs dys
  for (std::size_t i = 0; i < size; i++) {
    double x_delta = 0.0;
    double y_delta = 0.0;
    if (i == 0) {
      x_delta = points[i + 1].x() - points[i].x();
      y_delta = points[i + 1].y() - points[i].y();
    } else if (i == size - 1) {
      x_delta = points[i].x() - points[i - 1].x();
      y_delta = points[i].y() - points[i - 1].y();
    } else {
      x_delta = 0.5 * (points[i + 1].x() - points[i - 1].x());
      y_delta = 0.5 * (points[i + 1].y() - points[i - 1].y());
    }
    dxs.push_back(std::move(x_delta));
    dys.push_back(std::move(y_delta));
  }

  // headings
  for (std::size_t i = 0; i < size; i++) {
    headings->push_back(std::atan2(dys[i], dxs[i]));
  }

  // accu_s
  double dis = 0.0;
  accu_s->push_back(dis);
  for (std::size_t i = 1; i < size; i++) {
    dis += points[i].DistanceTo(points[i - 1]);
    accu_s->push_back(dis);
  }

  // x_over_s_first_derivatives  y_over_s_first_derivatives
  for (std::size_t i = 0; i < size; i++) {
    double xds = 0.0;
    double yds = 0.0;
    if (i == 0) {
      xds = (points[i + 1].x() - points[i].x()) /
            (accu_s->at(i + 1) - accu_s->at(i));
      yds = (points[i + 1].y() - points[i].y()) /
            (accu_s->at(i + 1) - accu_s->at(i));
    } else if (i == size - 1) {
      xds = (points[i].x() - points[i - 1].x()) /
            (accu_s->at(i) - accu_s->at(i - 1));
      yds = (points[i].y() - points[i - 1].y()) /
            (accu_s->at(i) - accu_s->at(i - 1));
    } else {
      xds = (points[i + 1].x() - points[i - 1].x()) /
            (accu_s->at(i + 1) - accu_s->at(i - 1));
      yds = (points[i + 1].y() - points[i - 1].y()) /
            (accu_s->at(i + 1) - accu_s->at(i - 1));
    }
    x_over_s_first_derivatives.push_back(std::move(xds));
    y_over_s_first_derivatives.push_back(std::move(yds));
  }

  // x_over_s_second_derivatives  y_over_s_second_derivatives
  for (std::size_t i = 0; i < size; i++) {
    double xdds = 0.0;
    double ydds = 0.0;
    if (i == 0) {
      xdds =
          (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
          (accu_s->at(i + 1) - accu_s->at(i));
      ydds =
          (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
          (accu_s->at(i + 1) - accu_s->at(i));
    } else if (i == size - 1) {
      xdds =
          (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
          (accu_s->at(i) - accu_s->at(i - 1));
      ydds =
          (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
          (accu_s->at(i) - accu_s->at(i - 1));
    } else {
      xdds = (x_over_s_first_derivatives[i + 1] -
              x_over_s_first_derivatives[i - 1]) /
             (accu_s->at(i + 1) - accu_s->at(i - 1));
      ydds = (y_over_s_first_derivatives[i + 1] -
              y_over_s_first_derivatives[i - 1]) /
             (accu_s->at(i + 1) - accu_s->at(i - 1));
    }
    x_over_s_second_derivatives.push_back(std::move(xdds));
    y_over_s_second_derivatives.push_back(std::move(ydds));
  }

  // x_over_s_third_derivatives  y_over_s_third_derivatives
  for (std::size_t i = 0; i < size; i++) {
    double xddds = 0.0;
    double yddds = 0.0;

    if (i == 0) {
      xddds = (x_over_s_second_derivatives[i + 1] -
               x_over_s_second_derivatives[i]) /
              (accu_s->at(i + 1) - accu_s->at(i));
      yddds = (y_over_s_second_derivatives[i + 1] -
               y_over_s_second_derivatives[i]) /
              (accu_s->at(i + 1) - accu_s->at(i));
    } else if (i == size - 1) {
      xddds = (x_over_s_second_derivatives[i] -
               x_over_s_second_derivatives[i - 1]) /
              (accu_s->at(i) - accu_s->at(i - 1));
      yddds = (y_over_s_second_derivatives[i] -
               y_over_s_second_derivatives[i - 1]) /
              (accu_s->at(i) - accu_s->at(i - 1));
    } else {
      xddds = (x_over_s_second_derivatives[i + 1] -
               x_over_s_second_derivatives[i - 1]) /
              (accu_s->at(i + 1) - accu_s->at(i - 1));
      yddds = (y_over_s_second_derivatives[i + 1] -
               y_over_s_second_derivatives[i - 1]) /
              (accu_s->at(i + 1) - accu_s->at(i - 1));
    }

    x_over_s_third_derivatives.push_back(std::move(xddds));
    y_over_s_third_derivatives.push_back(std::move(yddds));
  }

  // kappa
  for (std::size_t i = 0; i < size; i++) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    kappa->push_back(pnc::CurveMath::ComputeCurvature(xds, xdds, yds, ydds));
  }

  // dkappa
  for (std::size_t i = 0; i < size; i++) {
    double xds = x_over_s_first_derivatives[i];
    double yds = y_over_s_first_derivatives[i];
    double xdds = x_over_s_second_derivatives[i];
    double ydds = y_over_s_second_derivatives[i];
    double xddds = x_over_s_third_derivatives[i];
    double yddds = y_over_s_third_derivatives[i];
    dkappa->push_back(pnc::CurveMath::ComputeCurvatureDerivative(
        xds, xdds, xddds, yds, ydds, yddds));
  }

  // ddkappa
  for (std::size_t i = 0; i < size; i++) {
    double t_ddkappa = 0.0;
    if (i == 0) {
      t_ddkappa = (dkappa->at(i + 1) - dkappa->at(i)) /
                  (accu_s->at(i + 1) - accu_s->at(i));
    } else if (i == size - 1) {
      t_ddkappa = (dkappa->at(i) - dkappa->at(i - 1)) /
                  (accu_s->at(i) - accu_s->at(i - 1));
    } else {
      t_ddkappa = (dkappa->at(i + 1) - dkappa->at(i - 1)) /
                  (accu_s->at(i + 1) - accu_s->at(i - 1));
    }
    ddkappa->push_back(std::move(t_ddkappa));
  }
  return true;
}

std::vector<pose2d_t> LaneProcess::FitLaneMark(const pnc::LaneMark& lanemark,
                                               const double seg_dist,
                                               const double line_step,
                                               const double keep_dist) {
  std::vector<pose2d_t> result;
  std::vector<pnc::Point3D> last_input;

  // check duplicates id
  for (auto const& unit : tracking_units_) {
    if (lanemark.id() != unit.id) continue;
    if (seq_ == unit.seq && !unit.fit_output.empty()) {
      result = unit.fit_output;
      return result;
    }
    if (seq_ - 1 == unit.seq && FLAGS_enable_lane_mark_tracking) {
      last_input = unit.fit_input;
      PathTransform(unit.loc, em_loc_, &last_input);
    }
  }

  const auto& ori_pts = lanemark.pts_vehicle_coord();
  if (ori_pts.empty()) {
    return result;
  }

  // generate fit_data
  std::vector<pnc::Point3D> fit_data;
  if (last_input.empty()) {
    fit_data = std::vector<pnc::Point3D>(ori_pts.begin(), ori_pts.end());
  } else {
    auto fb = last_input.begin();
    auto fe = last_input.end();
    double min_d2 = std::numeric_limits<double>::max();
    bool find_fb = false;
    auto target = ori_pts[0];
    for (auto it = last_input.begin(); it != last_input.end(); ++it) {
      double tmp_d2 = (it->x() - target.x()) * (it->x() - target.x()) +
                      (it->y() - target.y()) * (it->y() - target.y());
      if (tmp_d2 <= min_d2) {
        min_d2 = tmp_d2;
        continue;
      }
      if (!find_fb) {
        fb = std::prev(it);
        find_fb = true;
        continue;
      }
      if (it->z() - fb->z() > keep_dist) {
        fe = it;
        break;
      }
    }
    if (min_d2 < 4.0) {
      fit_data = std::vector<pnc::Point3D>(fb, fe);
    }
    int fb_pos = 0;
    if (!fit_data.empty()) {
      min_d2 = std::numeric_limits<double>::max();
      target = fit_data.back();
      for (auto it = ori_pts.begin(); it != ori_pts.end(); ++it, ++fb_pos) {
        double tmp_d2 = (it->x() - target.x()) * (it->x() - target.x()) +
                        (it->y() - target.y()) * (it->y() - target.y());
        if (tmp_d2 > min_d2) {
          break;
        }
        min_d2 = tmp_d2;
      }
    }
    fit_data.insert(fit_data.end(), ori_pts.begin() + fb_pos, ori_pts.end());
  }

  double accu_s = 0.0;
  for (auto i = 0; i < fit_data.size(); ++i) {
    if (i > 0) {
      accu_s += std::hypot(fit_data[i].x() - fit_data[i - 1].x(),
                           fit_data[i].y() - fit_data[i - 1].y());
    }
    fit_data[i].set_z(accu_s);
  }

  // check if polynomial fitting can be used
  static constexpr double close_dist = 0.2;
  static constexpr double close_radio = 0.9;
  if (PolynomialAndCheck(fit_data, line_step, close_dist, close_radio, &result)) {
    TrackingUnit unit(lanemark.id(), seq_, em_loc_);
    unit.set_fit_input(std::move(fit_data));
    unit.set_fit_output(result);
    tracking_units_.emplace_back(std::move(unit));
    return result;
  }

  // cubic spline fitting
  static constexpr double smooth = 0.1;
  if (CubicSpline(fit_data, seg_dist, line_step, smooth, &result)) {
    TrackingUnit unit(lanemark.id(), seq_, em_loc_);
    unit.set_fit_input(std::move(fit_data));
    unit.set_fit_output(result);
    tracking_units_.emplace_back(std::move(unit));
    return result;
  }

  return result;
}

bool LaneProcess::CubicSpline(const std::vector<pnc::Point3D>& input,
                              const double seg_dist,
                              const double line_step,
                              const double smooth,
                              std::vector<pose2d_t>* const output,
                              ReferenceLine* const reference_line) {
  std::vector<pnc::Point3D> fit_data;
  size_t last_idx = 0;
  for (auto i = 0; i < input.size(); ++i) {
    if (i == 0 || input[i].z() - fit_data.back().z() >= seg_dist) {
      fit_data.emplace_back(input[i]);
      last_idx = i;
    }
  }
  if (fit_data.empty() || last_idx < 2) {
    return false;
  }

  auto node_num = fit_data.size();
  Eigen::VectorXd xdata(node_num);
  Eigen::VectorXd ydata(node_num);
  Eigen::VectorXd sdata(node_num);
  for (auto i = 0; i < node_num; ++i) {
    xdata(i) = fit_data[i].x();
    ydata(i) = fit_data[i].y();
    sdata(i) = fit_data[i].z();
  }

  // auto start_x = input.front().x();
  // auto start_y = input.front().y();
  // auto start_theta = std::atan2(input[1].y() - start_y, input[1].x() - start_x);
  // auto start_curvature = pnc::CurveMath::ComputeCurvature(
  //     pnc::Vec2d(start_x, start_y), pnc::Vec2d(input[1].x(), input[1].y()),
  //     pnc::Vec2d(input[2].x(), input[2].y()));
  // auto end_x = input[last_idx].x();
  // auto end_y = input[last_idx].y();
  // auto end_theta = std::atan2(end_y - input[last_idx - 1].y(),
  //                             end_x - input[last_idx - 1].x());
  // auto end_curvature = pnc::CurveMath::ComputeCurvature(
  //     pnc::Vec2d(input[last_idx - 2].x(), input[last_idx - 2].y()),
  //     pnc::Vec2d(input[last_idx - 1].x(), input[last_idx - 1].y()),
  //     pnc::Vec2d(end_x, end_y));
  // Eigen::VectorXd x_constraints(6), y_constraints(6);
  // x_constraints << start_x, std::cos(start_theta),
  //     -std::sin(start_theta) * start_curvature, end_x, std::cos(end_theta),
  //     -std::sin(end_theta) * end_curvature;
  // y_constraints << start_y, std::sin(start_theta),
  //     std::cos(start_theta) * start_curvature, end_y, std::sin(end_theta),
  //     std::cos(end_theta) * end_curvature;

  pnc::CubicSplineSmoother spline_sx, spline_sy;
  // spline_sx.init(sdata, xdata, smooth, x_constraints);
  // spline_sy.init(sdata, ydata, smooth, y_constraints);
  spline_sx.init(sdata, xdata, smooth);
  spline_sy.init(sdata, ydata, smooth);

  if (output) {
    for (double s = 0.0; s <= fit_data.back().z(); s += line_step) {
      output->emplace_back(
          std::make_pair(spline_sx.Evaluate(s), spline_sy.Evaluate(s)));
    }
    return true;
  }

  if (!reference_line) {
    return false;
  }

  auto get_kappa = [](Eigen::Vector3d const& x_dx_d2x,
                      Eigen::Vector3d const& y_dy_d2y) {
    return (x_dx_d2x(1) * y_dy_d2y(2) - y_dy_d2y(1) * x_dx_d2x(2)) /
           std::pow(x_dx_d2x(1) * x_dx_d2x(1) + y_dy_d2y(1) * y_dy_d2y(1), 1.5);
  };
  auto get_point = [&](double s, pnc::PathPoint* point) {
    size_t piece_idx = spline_sx.CalPieceIndex(s);
    Eigen::Vector3d x_dx_d2x = spline_sx.Evaluate(s, piece_idx);
    Eigen::Vector3d y_dy_d2y = spline_sy.Evaluate(s, piece_idx);
    point->set_x(x_dx_d2x(0));
    point->set_y(y_dy_d2y(0));
    point->set_s(s);
    point->set_theta(std::atan2(y_dy_d2y(1), x_dx_d2x(1)));
    point->set_kappa(get_kappa(x_dx_d2x, y_dy_d2y));
    auto front = s - 0.0001;
    piece_idx = spline_sx.CalPieceIndex(front);
    Eigen::Vector3d front_x_dx_d2x = spline_sx.Evaluate(front, piece_idx);
    Eigen::Vector3d front_y_dy_d2y = spline_sy.Evaluate(front, piece_idx);
    auto back = s + 0.0001;
    piece_idx = spline_sx.CalPieceIndex(back);
    Eigen::Vector3d back_x_dx_d2x = spline_sx.Evaluate(back, piece_idx);
    Eigen::Vector3d back_y_dy_d2y = spline_sy.Evaluate(back, piece_idx);
    double front_kappa = get_kappa(front_x_dx_d2x, front_y_dy_d2y);
    double back_kappa = get_kappa(back_x_dx_d2x, back_y_dy_d2y);
    point->set_dkappa((back_kappa - front_kappa) / 0.0002);
  };

  pnc::Path path;
  for (double s = 0.0; s <= fit_data.back().z(); s += line_step) {
    auto* point = path.add_path_point();
    get_point(s, point);
    if (s > FLAGS_max_navigation_length) {
      break;
    }
  }
  std::vector<pnc::PathPoint> path_points(path.mutable_path_point()->begin(),
                                          path.mutable_path_point()->end());
  reference_line->set_path_points(std::move(path_points));
  return true;
}

bool LaneProcess::PolynomialAndCheck(const std::vector<pnc::Point3D>& input,
                                     const double line_step,
                                     const double close_dist,
                                     const double close_radio,
                                     std::vector<pose2d_t>* const output,
                                     ReferenceLine* const reference_line) {
  Eigen::Matrix<float, 4, 1> coeff;
  std::vector<Eigen::Matrix<float, 2, 1>> xy_points;

  xy_points.resize(input.size());
  for (auto i = 0; i < input.size(); ++i) {
    xy_points[i](0) = input[i].x();
    xy_points[i](1) = input[i].y();
  }

  if (!pnc::CurveMath::PolyFit(xy_points, 3, &coeff)) {
    return false;
  }

  auto evaluate = [&](double x) {
    return EvaluateCubicPolynomial(coeff(0), coeff(1), coeff(2), coeff(3), x);
  };

  int close_pts_size =
      std::count_if(xy_points.begin(), xy_points.end(),
                    [&](Eigen::Matrix<float, 2, 1> const& pt) {
                      return std::abs(pt.y() - evaluate(pt.x())) < close_dist;
                    });

  if (FLAGS_use_line_pts &&
      static_cast<double>(close_pts_size) / xy_points.size() < close_radio) {
    return false;
  }

  if (output) {
    for (double x = input.front().x(); x < input.back().x(); x += line_step) {
      output->emplace_back(std::make_pair(x, evaluate(x)));
    }
    return true;
  }

  if (!reference_line) {
    return false;
  }

  pnc::Path path;
  pnc::CubicCurve ref_curve;
  ref_curve.set_c0(coeff(0));
  ref_curve.set_c1(coeff(1));
  ref_curve.set_c2(coeff(2));
  ref_curve.set_c3(coeff(3));
  ref_curve.set_start(input.front().x());
  ref_curve.set_end(input.back().x());
  double accumulated_s = 0;
  for (double x = ref_curve.start(); x < ref_curve.end(); x += line_step) {
    auto* point = path.add_path_point();
    if (!AddPathPoint(ref_curve, x, &accumulated_s, &path, point)) {
      break;
    }
  }
  std::vector<pnc::PathPoint> path_points(path.mutable_path_point()->begin(),
                                          path.mutable_path_point()->end());
  reference_line->set_path_points(std::move(path_points));
  return true;
}

void LaneProcess::PathTransform(const pnc::LocalizePose& from,
                                const pnc::LocalizePose& to,
                                std::vector<pose2d_t>* const path) {
  auto delta_x_1_2 = from.x - to.x;
  auto delta_y_1_2 = from.y - to.y;
  auto cos_0_2 = std::cos(-to.heading);
  auto sin_0_2 = std::sin(-to.heading);
  auto delta_x = delta_x_1_2 * cos_0_2 - delta_y_1_2 * sin_0_2;
  auto delta_y = delta_x_1_2 * sin_0_2 + delta_y_1_2 * cos_0_2;
  auto delta_heading = from.heading - to.heading;
  auto cos_1_2 = std::cos(delta_heading);
  auto sin_1_2 = std::sin(delta_heading);

  for (auto& pt : *path) {
    auto x = pt.first;
    auto y = pt.second;
    pt.first = delta_x + x * cos_1_2 - y * sin_1_2;
    pt.second = delta_y + x * sin_1_2 + y * cos_1_2;
  }
}

void LaneProcess::PathTransform(const pnc::LocalizePose& from,
                                const pnc::LocalizePose& to,
                                std::vector<pnc::Point3D>* const path) {
  std::vector<pose2d_t> tmp_path(path->size());
  for (auto i = 0; i < path->size(); ++i) {
    tmp_path[i].first = (*path)[i].x();
    tmp_path[i].second = (*path)[i].y();
  }

  PathTransform(from, to, &tmp_path);

  for (auto i = 0; i < path->size(); ++i) {
    (*path)[i].set_x(tmp_path[i].first);
    (*path)[i].set_y(tmp_path[i].second);
  }
}

}  // namespace pnc_map
}  // namespace xju
