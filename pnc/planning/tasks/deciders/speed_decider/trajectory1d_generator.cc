/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/tasks/deciders/speed_decider/trajectory1d_generator.h"

#include <cmath>
#include <utility>

#include "common/math/math_utils.h"
#include "common/math/path_matcher.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "planning/common/planning_gflags/planning_gflags.h"

namespace {
constexpr double kDoubleEpsilon = 1e-6;
}  // namespace

namespace xju {
namespace planning {

using PolynomialXOrder = pnc::PolynomialXOrder;
using State = std::array<double, 3>;           //(s,v,a)
using SampleState = std::pair<State, double>;  //((s,v,a),t)
using BoundaryType = STBoundary::BoundaryType;

Trajectory1dGenerator::Trajectory1dGenerator(
    const double cruising_speed, const SpeedDeciderConfig& speed_decider_config,
    const pnc::VehicleConfig& vehicle_config, const std::array<double, 3>& init_s,
    const PathData& path_data, const StGraphData& st_graph_data,
    const LaneChangeStatus& lane_change_status)
    : speed_decider_config_(speed_decider_config),
      vehicle_config_(vehicle_config),
      init_s_(init_s),
      path_data_(path_data),
      st_graph_data_(st_graph_data),
      lane_change_status_(lane_change_status),
      cruising_speed_(cruising_speed),
      feasible_region_(init_s) {}

bool Trajectory1dGenerator::GenerateLonTrajectoryBundle(
    std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
        lon_trajectory_bundle_ptr) {
  const auto& st_boundaries = st_graph_data_.st_boundaries();
  ADEBUG << "init state of plannint start point"
         << ", s: " << init_s_[0] << " ,v: " << init_s_[1]
         << ", a: " << init_s_[2];
  if (!GenerateSpeedProfilesForCruising(lon_trajectory_bundle_ptr)) {
    AERROR << "Failed to generate Cruising speed profiles.";
    return false;
  }
  if (!st_boundaries.empty()) {
    for (const auto* st_boundary_ptr : st_boundaries) {
      if (st_boundary_ptr->IsEmpty()) {
        continue;
      }
      if (st_boundary_ptr->boundary_type() != BoundaryType::UNKNOWN) {
        GenerateSpeedProfilesWithDecision(*st_boundary_ptr,
                                          lon_trajectory_bundle_ptr);
      } else {
        GenerateSpeedProfilesForFollow(*st_boundary_ptr,
                                       lon_trajectory_bundle_ptr);
        GenerateSpeedProfilesForOvertake(*st_boundary_ptr,
                                         lon_trajectory_bundle_ptr);
      }
    }
  }
  return true;
}

bool Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
        lon_trajectory_bundle_ptr) {
  size_t init_size = lon_trajectory_bundle_ptr->size();
  if (cruising_speed_ < -1e-3) {
    return false;
  }
  std::vector<double> t_samples;
  double t_min = 3.0;
  double t_step = speed_decider_config_.time_sample_resolution();
  for (double t = t_min; t <= FLAGS_trajectory_time_length; t += t_step) {
    t_samples.push_back(t);
  }

  std::vector<SampleState> end_sample_states;
  for (size_t j = 0; j < t_samples.size(); ++j) {
    double t = t_samples[j];
    end_sample_states.emplace_back(
        std::pair<State, double>({0.0, init_s_[1], 0.0}, t));
    end_sample_states.emplace_back(
        std::pair<State, double>({0.0, cruising_speed_, 0.0}, t));
    double v_upper = std::fmin(feasible_region_.VUpper(t), cruising_speed_);
    double v_lower = std::fmax(0.0, feasible_region_.VLower(t));
    static constexpr size_t num_of_v_sample = 4;
    double v_step = (v_upper - v_lower) / static_cast<double>(num_of_v_sample);

    v_step = std::fmin(speed_decider_config_.min_velocity_resolution(), v_step);
    for (double v_tmp = v_upper; v_tmp >= v_lower; v_tmp -= v_step) {
      end_sample_states.emplace_back(
          std::pair<State, double>({0.0, v_tmp, 0.0}, t));
    }
  }

  for (const auto& end_sample_state : end_sample_states) {
    const auto& s_poly = std::make_shared<PolynomialXOrder>();
    s_poly->set_label("Cruising");
    CalculateQuarticPolyCurve(
        init_s_[0], init_s_[1], init_s_[2], end_sample_state.first[1],
        end_sample_state.first[2], end_sample_state.second, s_poly.get());
    lon_trajectory_bundle_ptr->emplace_back(s_poly);
  }
  size_t end_size = lon_trajectory_bundle_ptr->size();
  ADEBUG << "Speed sample size For Cruising is: " << end_size - init_size;
  return true;
}

void Trajectory1dGenerator::GenerateSpeedProfilesWithDecision(
    const STBoundary& st_boundary,
    std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
        lon_trajectory_bundle_ptr) {
  if (st_boundary.boundary_type() == BoundaryType::STOP) {
    has_stop_point_ = true;
    stop_point_s_ = st_boundary.bottom_left_point().s();
    GenerateSpeedProfilesForStopping(st_boundary, lon_trajectory_bundle_ptr);
  } else if (st_boundary.boundary_type() == BoundaryType::FOLLOW) {
    GenerateSpeedProfilesForFollow(st_boundary, lon_trajectory_bundle_ptr);
  } else if (st_boundary.boundary_type() == BoundaryType::OVERTAKE) {
    GenerateSpeedProfilesForOvertake(st_boundary, lon_trajectory_bundle_ptr);
  } else {
    ADEBUG << "error boundary type ";
  }
}

void Trajectory1dGenerator::GenerateSpeedProfilesForStopping(
    const STBoundary& st_boundary,
    std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
        lon_trajectory_bundle_ptr) {
  size_t init_size = lon_trajectory_bundle_ptr->size();
  static constexpr size_t num_of_t_sample = 9;
  std::array<double, num_of_t_sample> t_samples;
  for (size_t i = 1; i < num_of_t_sample; ++i) {
    auto ratio =
        static_cast<double>(i) / static_cast<double>(num_of_t_sample - 1);
    t_samples[i] = FLAGS_trajectory_time_length * ratio;
  }
  t_samples[0] = speed_decider_config_.min_sample_time();
  for (const auto& t : t_samples) {
    double end_s = st_boundary.bottom_left_point().s() -
                   speed_decider_config_.lon_stop_distance_buffer();
    const auto& s_poly = std::make_shared<PolynomialXOrder>();
    s_poly->set_label("Stop");
    CalculateQuinticPolyCurve(init_s_[0], init_s_[1], init_s_[2], end_s,
                              0.0, 0.0, t, s_poly.get());
    lon_trajectory_bundle_ptr->emplace_back(s_poly);
  }
  size_t end_size = lon_trajectory_bundle_ptr->size();
  ADEBUG << "Speed sample size For Stop is: " << end_size - init_size;
  return;
}

void Trajectory1dGenerator::GenerateSpeedProfilesForFollow(
    const STBoundary& st_boundary,
    std::vector<std::shared_ptr<PolynomialXOrder>>* const
        lon_trajectory_bundle_ptr) {
  size_t init_size = lon_trajectory_bundle_ptr->size();
  std::vector<STPoint> st_points;
  const auto& speed_limit = st_graph_data_.speed_limit();
  if (GetStBoundarySurroundingPoints(
          st_boundary, true, speed_decider_config_.time_sample_resolution(), &st_points)) {
    double limit_speed = cruising_speed_;
    ADEBUG << "Get StBoundary Surrounding Points size is: " << st_points.size();
    for (const auto& st_point : st_points) {
      double t = st_point.t();
      double v_lower = 0.0;
      double v_upper = 0.0;
      st_boundary.GetBoundarySlopes(t, v_lower, v_upper);

      double s_upper =
          st_point.s() - speed_decider_config_.lon_follow_distance_buffer();
      s_upper = std::fmin(feasible_region_.SUpper(t), s_upper);
      double s_lower =
          s_upper - speed_decider_config_.default_s_sample_buffer();
      s_lower = std::fmax(feasible_region_.SLower(t), s_lower);
      if (s_upper - s_lower < 1e-3) {
        continue;
      }
      // ADEBUG << "easible_region_.SLower: " << feasible_region_.SLower(t) <<
      // ", feasible_region_.SUpper(t): " << feasible_region_.SUpper(t); ADEBUG
      // << "s_lower: " << s_lower << ", s_upper: " << s_upper;
      for (double end_s = s_lower; end_s <= s_upper;
           end_s += speed_decider_config_.min_s_resolution()) {
        if (speed_decider_config_.enable_speed_limit()) {
          limit_speed = speed_limit.GetSpeedLimitByS(end_s);
        }
        double end_v = std::fmin((v_lower + v_upper) / 2.0, limit_speed);
        const auto& s_poly = std::make_shared<PolynomialXOrder>();
        s_poly->set_label("Follow");
        CalculateQuinticPolyCurve(init_s_[0], init_s_[1], init_s_[2], end_s,
                                  end_v, 0.0, t, s_poly.get());
        lon_trajectory_bundle_ptr->emplace_back(s_poly);
      }
    }
  }
  size_t end_size = lon_trajectory_bundle_ptr->size();
  ADEBUG << "Speed sample size For Follow is: " << end_size - init_size;
  return;
}

void Trajectory1dGenerator::GenerateSpeedProfilesForOvertake(
    const STBoundary& st_boundary,
    std::vector<std::shared_ptr<pnc::PolynomialXOrder>>* const
        lon_trajectory_bundle_ptr) {
  size_t init_size = lon_trajectory_bundle_ptr->size();
  std::vector<STPoint> st_points;
  const auto& speed_limit = st_graph_data_.speed_limit();
  if (GetStBoundarySurroundingPoints(
          st_boundary, false, speed_decider_config_.time_sample_resolution(), &st_points)) {
    double limit_speed = cruising_speed_;
    for (const auto& st_point : st_points) {
      double t = st_point.t();
      double v_lower = 0.0;
      double v_upper = 0.0;
      st_boundary.GetBoundarySlopes(t, v_lower, v_upper);
      double s_lower =
          st_point.s() + speed_decider_config_.lon_overtake_distance_buffer();
      s_lower = std::fmax(feasible_region_.SLower(t), s_lower);
      double s_upper =
          s_lower + speed_decider_config_.default_s_sample_buffer();
      s_upper = std::min(
          {feasible_region_.SUpper(t), s_upper, st_graph_data_.path_length()});

      if (s_upper - s_lower < 1e-3) {
        continue;
      }
      for (double end_s = s_lower; end_s <= s_upper;
           end_s += speed_decider_config_.min_s_resolution()) {
        if (speed_decider_config_.enable_speed_limit()) {
          limit_speed = speed_limit.GetSpeedLimitByS(end_s);
        }
        double end_v = std::fmin((v_lower + v_upper) / 2.0, limit_speed);
        const auto& s_poly = std::make_shared<PolynomialXOrder>();
        s_poly->set_label("Overtake");
        CalculateQuinticPolyCurve(init_s_[0], init_s_[1], init_s_[2], end_s,
                                  end_v, 0.0, t, s_poly.get());
        lon_trajectory_bundle_ptr->emplace_back(s_poly);
      }
    }
  }
  size_t end_size = lon_trajectory_bundle_ptr->size();
  ADEBUG << "Speed sample size For overtake is: " << end_size - init_size;
  return;
}

bool Trajectory1dGenerator::GetStBoundarySurroundingPoints(
    const STBoundary& st_boundary, const bool is_lower_bound,
    const double t_min_density, std::vector<STPoint>* const st_points) {
  st_points->clear();
  double t_0 = 0.0, t_1 = 0.0;
  if (!is_lower_bound) {
    t_0 = st_boundary.upper_left_point().t();
    t_1 = st_boundary.upper_right_point().t();
  } else {
    t_0 = st_boundary.bottom_left_point().t();
    t_1 = st_boundary.bottom_right_point().t();
  }
  double t_gap = t_1 - t_0;
  if (t_gap < 1e-6) {
    return false;
  }
  for (double t = t_0; t <= t_1; t += t_min_density) {
    double s_lower = 0, s_upper = 0;
    st_boundary.GetBoundarySRange(t, s_lower, s_upper);
    double s = is_lower_bound ? s_lower : s_upper;
    st_points->emplace_back(s, t);
  }
  return true;
}

bool Trajectory1dGenerator::TrajectoryEvaluate(
    const std::vector<std::shared_ptr<PolynomialXOrder>>& lon_trajectory_bundle,
    std::shared_ptr<PolynomialXOrder> final_poly_ptr) {
  if (lon_trajectory_bundle.empty()) {
    AERROR << "lon_trajectory_bundle is empty";
    return false;
  }
  if (has_stop_point_) {
    set_weights(speed_decider_config_.has_stop_decision_weights());
  } else if (lane_change_status_.status() ==
             LaneChangeStatus::LANE_CHANGE_PREPARE) {
    set_weights(speed_decider_config_.lane_change_prepare_weights());
  } else if (st_graph_data_.st_boundaries().size() == 0) {
    set_weights(speed_decider_config_.default_weights());
  } else {
    set_weights(speed_decider_config_.has_obs_weights());
  }
  const auto& ref_s_dot = ComputeGuideVelocity();
  CalculateLongitudinalBoundary();
  std::vector<std::pair<double, std::shared_ptr<PolynomialXOrder>>>
      cost_sample_poly_vec;
  for (const auto& lon_trajectory : lon_trajectory_bundle) {
    // ADEBUG << "sample end state: " << lon_trajectory->end_state().first[0]
    //        << " ," << lon_trajectory->end_state().first[1] << " ,"
    //        << lon_trajectory->end_state().first[2]
    //        << " . sample end time: " << lon_trajectory->end_state().second
    //        << " s.";
    double cost_tmp = CalculateTrajectoryCost(ref_s_dot, lon_trajectory);
    if (cost_tmp > 1e-3) {
      cost_sample_poly_vec.push_back(
          std::pair<double, std::shared_ptr<PolynomialXOrder>>(cost_tmp,
                                                               lon_trajectory));
    }
  }

  ADEBUG << "cost_sample_poly_vec size is: " << cost_sample_poly_vec.size();
  if (!cost_sample_poly_vec.empty()) {
    std::sort(cost_sample_poly_vec.begin(), cost_sample_poly_vec.end(),
              [](std::pair<double, std::shared_ptr<PolynomialXOrder>> s1,
                 std::pair<double, std::shared_ptr<PolynomialXOrder>> s2) {
                return std::get<0>(s1) < std::get<0>(s2);
              });
    *final_poly_ptr = *(cost_sample_poly_vec.front().second);
    ADEBUG << "final polynomial end state: s = "
           << final_poly_ptr->end_state().first[0]
           << "m ,v= " << final_poly_ptr->end_state().first[1]
           << "m/s ,t = " << final_poly_ptr->end_state().second
           << "s, label = " << final_poly_ptr->label();
    return true;
  }
  return false;
}

void Trajectory1dGenerator::CalculateLongitudinalBoundary() {
  constexpr static double kBoundayScaleFactor = 1.1;
  acceleration_lower_bound_ =
      std::fmin(FLAGS_longitudinal_acceleration_lower_bound, init_s_[2]) *
      kBoundayScaleFactor;
  acceleration_upper_bound_ =
      std::fmax(FLAGS_longitudinal_acceleration_upper_bound, init_s_[2]) *
      kBoundayScaleFactor;
  jerk_lower_bound_ = FLAGS_longitudinal_jerk_lower_bound * kBoundayScaleFactor;
  jerk_upper_bound_ = FLAGS_longitudinal_jerk_upper_bound * kBoundayScaleFactor;
}

double Trajectory1dGenerator::CalculateTrajectoryCost(
    const std::vector<std::pair<double, double>>& ref_s_dot,
    const std::shared_ptr<pnc::PolynomialXOrder>& s_poly_ptr) {
  if (!s_poly_ptr) {
    return -1.0;
  }
  const auto& speed_limit = st_graph_data_.speed_limit();

  double traj_s = 0.0, traj_v = 0.0, traj_a = 0.0, traj_jerk = 0.0;
  double end_sample_t = s_poly_ptr->end_state().second;
  double collision_dist_cost_sqr_sum = 0.0;
  double collision_dist_cost_abs_sum = 0.0;
  double sigma = speed_decider_config_.lon_dist_cost_std();
  double speed_cost_sqr_sum = 0.0;
  double speed_cost_weight_sum = 0.0;
  double acc_cost_sqr_sum = 0.0;
  double acc_cost_abs_sum = 0.0;
  double centripetal_acc_sum = 0.0;
  double centripetal_acc_sqr_sum = 0.0;
  double jerk_cost_sqr_sum = 0.0;
  double jerk_cost_abs_sum = 0.0;
  double follow_cost_sqr_sum = 0.0;
  double follow_cost_weight_sum = 0.0;

  for (size_t i = 0; i < ref_s_dot.size(); ++i) {
    double t = ref_s_dot[i].first;
    // calculate s, v, a, jerk
    if (t <= end_sample_t) {
      traj_s = s_poly_ptr->Eval(0, t);
      traj_v = s_poly_ptr->Eval(1, t);
      traj_a = s_poly_ptr->Eval(2, t);
      traj_jerk = s_poly_ptr->Eval(3, t);
    } else {
      traj_s += traj_v * (ref_s_dot[i].first - ref_s_dot[i - 1].first);
      traj_a = 0;
      traj_jerk = 0;
    }

    // double limit_speed = std::fmin(cruising_speed_, speed_limit.GetSpeedLimitByS(traj_s));
    // double min_v_constraint = MinVelocityInKinematicConstraints(t, init_s_[1], init_s_[2]);

    // double speed_upper_bound = std::fmax(limit_speed, min_v_constraint) * 1.1;
    if (traj_s < s_lower_bound_ || traj_v < speed_lower_bound_) {
      // ADEBUG << "traj_s or  traj_v is out of the range,"
      //        << " t: " << t << "s, traj_s: " << traj_s << "m, traj_v: " <<
      //        traj_v;
      return -1.0;
    }
    // calculate acc_cost
    if (traj_a < acceleration_lower_bound_ ||
        traj_a > acceleration_upper_bound_) {
      // ADEBUG << "acceleration is out of the range,"
      //        << " t: " << t << "s, acc: " << traj_a;
      return -1.0;
    }
    double traj_a_cost = traj_a / FLAGS_longitudinal_acceleration_upper_bound;
    acc_cost_sqr_sum += traj_a_cost * traj_a_cost;
    acc_cost_abs_sum += std::fabs(traj_a_cost);

    // calculate centeripetal_acc_cost
    const auto& match_point =
        pnc::PathMatcher::MatchToPath(path_data_.planned_path(), traj_s);
    if (match_point.has_kappa()) {
      double centripetal_acc = traj_v * traj_v * match_point.kappa();
      double centripetal_acc_cost =
          std::fabs(centripetal_acc / FLAGS_max_centripetal_acceleration);
      centripetal_acc_sum += std::fabs(centripetal_acc_cost);
      centripetal_acc_sqr_sum += centripetal_acc_cost * centripetal_acc_cost;
    }

    // calculate jerk_cost
    if (traj_jerk < jerk_lower_bound_ || traj_jerk > jerk_upper_bound_) {
      // ADEBUG << "jerk is out of the range,"
      //        << " t: " << t << "s, jerk: " << traj_jerk;
      return -1.0;
    }
    double traj_jerk_cost =
        traj_jerk / FLAGS_longitudinal_acceleration_upper_bound;
    jerk_cost_sqr_sum += traj_jerk_cost * traj_jerk_cost;
    jerk_cost_abs_sum += std::fabs(traj_jerk_cost);

    // calculate guide velocity cost
    double delta_v = 0.0;

    if (speed_decider_config_.enable_speed_limit()) {
      traj_s = std::fmax(0.0, traj_s);
      double limit_speed =
          st_graph_data_.speed_limit().GetSpeedLimitByS(traj_s);
      delta_v = std::fmin(ref_s_dot[i].second, limit_speed) - traj_v;
    } else {
      delta_v = ref_s_dot[i].second - traj_v;
    }
    speed_cost_sqr_sum += t * t * std::fabs(delta_v);
    speed_cost_weight_sum += t * t;

    // calculate distance cost and follow cost
    double thw = 2.5;
    double safe_distance = 3.0;

    for (const auto* st_boundary_ptr : st_graph_data_.st_boundaries()) {
      double obs_s_lower = st_graph_data_.path_length();
      double obs_s_upper = 0.0;
      double obs_btm_left_t = st_boundary_ptr->bottom_left_point().t();
      double obs_btm_left_s = st_boundary_ptr->bottom_left_point().s();
      double obs_up_left_s = st_boundary_ptr->upper_left_point().s();
      double traj_s_min_t = s_poly_ptr->Eval(0, obs_btm_left_t);
      double desired_dist = traj_v * thw + safe_distance;
      double follow_dist = 200, collision_dist = 0.0;
      if (st_boundary_ptr->GetBoundarySRange(t, obs_s_lower, obs_s_upper)) {
        if (traj_s_min_t < obs_btm_left_s && traj_s < obs_s_lower) {
          follow_dist = obs_s_lower - traj_s;
          collision_dist = follow_dist;
        } else if (traj_s_min_t > obs_up_left_s && traj_s > obs_s_upper) {
          follow_dist = traj_s - obs_s_upper;
          collision_dist = follow_dist;
        }
        double error_desired_dist = std::fabs(follow_dist - desired_dist);
        double collosion_dist_cost =
            std::exp(-collision_dist * collision_dist / (2.0 * sigma * sigma));
        collision_dist_cost_sqr_sum +=
            collosion_dist_cost * collosion_dist_cost;
        collision_dist_cost_abs_sum += collosion_dist_cost;

        follow_cost_sqr_sum += t * t * error_desired_dist;
        follow_cost_weight_sum += t * t;
        // ADEBUG << "t: " << t
        //        << ", distance between traj_s and obs is : " << dist
        //        << " , and collosion_dist_cost is: " << collosion_dist_cost
        //        << " , collision_dist_cost_sqr_sum is: "
        //        << collision_dist_cost_sqr_sum
        //        << " , collision_dist_cost_abs_sumis: "
        //        << collision_dist_cost_abs_sum;
      }
    }
  }
  double collision_dist_cost_sum =
      collision_dist_cost_sqr_sum /
      (collision_dist_cost_abs_sum + kDoubleEpsilon) * weight_collision_dist_;
  // double collision_dist_cost_sum = 0.0;
  double follow_dist_cost_sum = follow_cost_sqr_sum /
                                (follow_cost_weight_sum + kDoubleEpsilon) *
                                weight_follow_dist_;
  double speed_cost_sum = speed_cost_sqr_sum /
                          (speed_cost_weight_sum + kDoubleEpsilon) *
                          weight_guide_velocity_;
  double acc_cost_sum = acc_cost_sqr_sum / (acc_cost_abs_sum + kDoubleEpsilon) *
                        weight_lon_accelerate_;
  double centripetal_acc_cost_sum = centripetal_acc_sqr_sum /
                                    (centripetal_acc_sum + kDoubleEpsilon) *
                                    weight_centripetal_acc_;
  double jerk_cost_sum = jerk_cost_sqr_sum /
                         (jerk_cost_abs_sum + kDoubleEpsilon) *
                         weight_lon_jerk_;
  double cost_sum = collision_dist_cost_sum + follow_dist_cost_sum +
                    centripetal_acc_cost_sum + speed_cost_sum + acc_cost_sum +
                    jerk_cost_sum;

  // ADEBUG << "cost_sum: " << cost_sum
  //        << " , collision_dist_cost: " << collision_dist_cost_sum
  //        << " , follow_dist_cost_sum: " << follow_dist_cost_sum
  //        << " , lon_velocity_cost: " << speed_cost_sum
  //        << " , lon_acce_cost: " << acc_cost_sum
  //        << " , centripetal_acc_cost: " << centripetal_acc_cost_sum
  //        << " , lon_jerk_cost: " << jerk_cost_sum;
  return cost_sum;
}

double Trajectory1dGenerator::MinVelocityInKinematicConstraints(double t, double init_v, double init_a) {
  if (init_a <= FLAGS_longitudinal_acceleration_lower_bound) {
    double v = init_v + FLAGS_longitudinal_acceleration_lower_bound * t;
    return v >= 0 ? v : 0.0;
  }
  double temp_t = (init_a - FLAGS_longitudinal_acceleration_lower_bound) /
                  (-FLAGS_longitudinal_jerk_lower_bound);
  if (temp_t >= t) {
    double v =
        init_v + init_a * t + 0.5 * FLAGS_longitudinal_jerk_lower_bound * t * t;
    return v >= 0 ? v : 0.0;
  }

  double v = init_v + init_a * temp_t +
             0.5 * FLAGS_longitudinal_jerk_lower_bound * temp_t * temp_t;
  v = v + FLAGS_longitudinal_acceleration_lower_bound * (t - temp_t);
  return v >= 0 ? v : 0.0;
}
bool Trajectory1dGenerator::IsCollision(
    const std::shared_ptr<PolynomialXOrder>& s_poly_ptr) {
  if (!s_poly_ptr) {
    AERROR << "s_poly_ptr is nullptr";
    return true;
  }
  double thw = 2;
  double follow_distance = init_s_[1] * thw + 3.0;
  for (double t = 0.1; t <= FLAGS_trajectory_time_length;
       t += speed_decider_config_.dense_time_resolution()) {
    double s = s_poly_ptr->Eval(0, t);
    for (const auto* st_boundary_ptr : st_graph_data_.st_boundaries()) {
      double obs_s_lower = st_graph_data_.path_length();
      double obs_s_upper = 0.0;
      if (st_boundary_ptr->GetBoundarySRange(t, obs_s_lower, obs_s_upper) &&
          s >= (obs_s_lower - follow_distance) &&
          s <= (obs_s_upper + follow_distance)) {
        ADEBUG << "has collision with obs: " << st_boundary_ptr->id()
               << ", at t: " << t;
        return true;
      }
    }
  }
  return false;
}

std::vector<std::pair<double, double>>
Trajectory1dGenerator::ComputeGuideVelocity() {
  std::vector<std::pair<double, double>> ref_s_dot;
  // init_t, end_t, acceleration
  std::vector<std::tuple<double, double, ConstantAccelerationModelT*>>
      piecewise_motions;
  double start_t = 0.0;
  if (!has_stop_point_ &&
      lane_change_status_.status() != LaneChangeStatus::LANE_CHANGE_PREPARE) {
    ConstantAccelerationModelT* motion = new ConstantAccelerationModelT();
    double acc = (cruising_speed_ - init_s_[1]) / FLAGS_trajectory_time_length;
    acc =
        pnc::Clamp(acc, FLAGS_comfortable_longitudinal_acceleration_lower_bound,
                   FLAGS_comfortable_longitudinal_acceleration_upper_bound);
    ADEBUG << "ego cruising motion init, v = " << init_s_[1] << " ,a = " << acc;
    motion->Init(0.0, init_s_[1], acc);
    piecewise_motions.emplace_back(0.0, FLAGS_trajectory_time_length, motion);
  } else if (!has_stop_point_ && lane_change_status_.status() ==
                                     LaneChangeStatus::LANE_CHANGE_PREPARE) {
    // calculate accelerate motion
    double acc_cmd = lane_change_status_.prepare_acceleration();
    double accelerate_time =
        (lane_change_status_.prepare_speed() - init_s_[1]) /
        (acc_cmd + kDoubleEpsilon);
    if (accelerate_time > speed_decider_config_.dense_time_resolution()) {
      ConstantAccelerationModelT* acc_motion = new ConstantAccelerationModelT();
      acc_motion->Init(0.0, init_s_[1], acc_cmd);

      piecewise_motions.emplace_back(
          0.0, std::fmin(FLAGS_trajectory_time_length, accelerate_time),
          acc_motion);
      ADEBUG << "ego accelerate motion init, v = " << init_s_[1]
             << " ,a = " << acc_cmd << " ,end_t = "
             << std::fmin(FLAGS_trajectory_time_length, accelerate_time);
    } else {
      accelerate_time = 0.0;
    }

    // calculate uniform motion
    if (accelerate_time < FLAGS_trajectory_time_length) {
      double acc_motion_end_s = init_s_[1] * accelerate_time +
                                acc_cmd * accelerate_time * accelerate_time / 2;
      double acc_motion_end_v = lane_change_status_.prepare_speed();
      ConstantAccelerationModelT* uniform_spd =
          new ConstantAccelerationModelT();

      uniform_spd->Init(acc_motion_end_s, acc_motion_end_v, 0.0);

      piecewise_motions.emplace_back(accelerate_time,
                                     FLAGS_trajectory_time_length, uniform_spd);
      ADEBUG << "ego uniform_spd motion init, v = " << acc_motion_end_v
             << " ,a = " << 0.0
             << " ,end_t = " << FLAGS_trajectory_time_length;
    }

  } else {
    GenerateStopPiecewisMotions(&piecewise_motions);
  }

  double t = 0.0;
  for (const auto& piecewise_motion : piecewise_motions) {
    double init_t = std::get<0>(piecewise_motion);
    double end_t = std::get<1>(piecewise_motion);
    auto* const_acc_motion = std::get<2>(piecewise_motion);
    for (t = init_t; t < end_t;
         t += speed_decider_config_.dense_time_resolution()) {
      ref_s_dot.emplace_back(t, const_acc_motion->EvaluateVelocity(t - init_t));
    }
  }
  for (int i = 0; i < piecewise_motions.size(); ++i) {
    delete std::get<2>(piecewise_motions[i]);
  }
  return ref_s_dot;
};

void Trajectory1dGenerator::GenerateStopPiecewisMotions(
    std::vector<std::tuple<double, double, ConstantAccelerationModelT*>>* const
        piecewise_motions_ptr) {
  double d_comfort = FLAGS_comfortable_longitudinal_acceleration_lower_bound;
  double stop_s =
      stop_point_s_ - speed_decider_config_.lon_stop_distance_buffer();
  double comfort_stop_dist =
      -init_s_[1] * init_s_[1] / (2 * d_comfort - kDoubleEpsilon);
  if (comfort_stop_dist > stop_s || (init_s_[1] < 1.0 && stop_s < 10)) {
    double safe_dist = std::fmax(0.0, stop_s);
    double d_cmd = -init_s_[1] * init_s_[1] / (2 * safe_dist + kDoubleEpsilon);
    d_cmd = std::fmin(d_cmd, d_comfort);
    ConstantAccelerationModelT* motion = new ConstantAccelerationModelT();
    ADEBUG << "ego uncomfort dec motion init, v = " << init_s_[1]
           << " ,a = " << d_cmd << " ,end_t = " << -init_s_[1] / d_comfort;
    motion->Init(0.0, init_s_[1], d_cmd);
    piecewise_motions_ptr->emplace_back(0.0, FLAGS_trajectory_time_length,
                                        motion);
    return;
  }
  if (init_s_[1] > cruising_speed_) {
    
    double start_cruise_t = (cruising_speed_ - init_s_[1]) / (d_comfort - kDoubleEpsilon);
    if (start_cruise_t >= speed_decider_config_.dense_time_resolution()) {
      ConstantAccelerationModelT* dec2cruise_motion =
          new ConstantAccelerationModelT();
      dec2cruise_motion->Init(0.0, init_s_[1], d_comfort);
      ADEBUG << "ego decelrate to cruising speed init, v = " << init_s_[1]
             << " ,a = " << d_comfort << " ,end_t = " << start_cruise_t;
      piecewise_motions_ptr->emplace_back(0, start_cruise_t, dec2cruise_motion);
    } else {
      start_cruise_t = 0.0;
    }
    double start_dec_t =
        start_cruise_t +
        (stop_s + init_s_[1] * init_s_[1] / (2 * d_comfort - kDoubleEpsilon)) /
            (cruising_speed_ + kDoubleEpsilon);
    if (start_dec_t >= speed_decider_config_.dense_time_resolution()) {
      ConstantAccelerationModelT* cruising_spd =
          new ConstantAccelerationModelT();
      cruising_spd->Init(0.0, cruising_speed_, 0.0);
      ADEBUG << "ego uniform_spd motion init, v = " << cruising_speed_
             << " ,a = " << 0.0 << " ,end_t = " << start_dec_t;
      piecewise_motions_ptr->emplace_back(start_cruise_t, start_dec_t,
                                          cruising_spd);
    } else {
      start_dec_t = 0.0;
    }
    
    double dec2zero_t = start_dec_t - cruising_speed_ / (d_comfort - kDoubleEpsilon);
    ConstantAccelerationModelT* dec2zero_motion = new ConstantAccelerationModelT();
    dec2zero_motion->Init(0.0, cruising_speed_, d_comfort);
    ADEBUG << "ego comfort decelrate to zero motion init, v = " << init_s_[1]
           << " ,a = " << d_comfort
           << " ,end_t: " << dec2zero_t;
    piecewise_motions_ptr->emplace_back(
        start_dec_t, FLAGS_trajectory_time_length, dec2zero_motion);
    return;

  }

  double start_dec_t =
      (stop_s - comfort_stop_dist) / (init_s_[1] + kDoubleEpsilon);

  if (start_dec_t < speed_decider_config_.max_start_dec_t()) {
    ConstantAccelerationModelT* uniform_spd = new ConstantAccelerationModelT();
    uniform_spd->Init(0.0, init_s_[1], 0.0);
    ADEBUG << "ego uniform_spd motion init, v = " << init_s_[1] << " ,a = " << 0.0
           << " ,end_t = " << start_dec_t;
    piecewise_motions_ptr->emplace_back(0, start_dec_t, uniform_spd);
    ConstantAccelerationModelT* dec_motion = new ConstantAccelerationModelT();
    dec_motion->Init(stop_s - comfort_stop_dist, init_s_[1], d_comfort);
    ADEBUG << "ego comfort dec motion init, v = " << init_s_[1]
           << " ,a = " << d_comfort
           << " ,end_t: " << start_dec_t - init_s_[1] / d_comfort;
    piecewise_motions_ptr->emplace_back(
        start_dec_t, FLAGS_trajectory_time_length, dec_motion);
    return;
  } else {
    double a_comfort = FLAGS_comfortable_longitudinal_acceleration_upper_bound;
    double dec_dist = (2 * a_comfort * stop_s + init_s_[1] * init_s_[1]) /
                      (2 * a_comfort - 2 * d_comfort + kDoubleEpsilon);

    double v_1 = std::sqrt(-2 * d_comfort * dec_dist);
    if (v_1 < cruising_speed_) {
      start_dec_t = (v_1 - init_s_[1]) / (a_comfort + kDoubleEpsilon);
      ConstantAccelerationModelT* acc_motion = new ConstantAccelerationModelT();
      acc_motion->Init(0.0, init_s_[1], a_comfort);
      ADEBUG << "ego comfort acc motion init, v = " << init_s_[1]
             << " ,a = " << a_comfort << " , end_t = " << start_dec_t;
      piecewise_motions_ptr->emplace_back(0, start_dec_t, acc_motion);
      ConstantAccelerationModelT* dec_motion = new ConstantAccelerationModelT();
      dec_motion->Init(stop_s - dec_dist, v_1, d_comfort);
      ADEBUG << "comfort dec motion init, v = " << v_1 << " ,a = " << d_comfort
             << " ,end_t: " << start_dec_t - v_1 / d_comfort;
      piecewise_motions_ptr->emplace_back(
          start_dec_t, FLAGS_trajectory_time_length, dec_motion);

    } else {
      // constant acceleration model
      double acc_dist =
          (cruising_speed_ * cruising_speed_ - init_s_[1] * init_s_[1]) /
          (2 * a_comfort + kDoubleEpsilon);
      double start_cruise_t =
          (cruising_speed_ - init_s_[1]) / (a_comfort + kDoubleEpsilon);
      ConstantAccelerationModelT* acc_motion = new ConstantAccelerationModelT();
      acc_motion->Init(0.0, init_s_[1], a_comfort);
      ADEBUG << "ego comfort acc motion init, v = " << init_s_[1]
             << " ,a = " << a_comfort << " , end_t = " << start_cruise_t;
      piecewise_motions_ptr->emplace_back(0, start_cruise_t, acc_motion);
      // cruising model
      dec_dist = (-cruising_speed_ * cruising_speed_) /
                 (2 * d_comfort - kDoubleEpsilon);
      double cruise_dist = stop_s - acc_dist - dec_dist;
      start_dec_t =
          start_cruise_t + cruise_dist / (cruising_speed_ + kDoubleEpsilon);
      ConstantAccelerationModelT* cruise_motion =
          new ConstantAccelerationModelT();
      cruise_motion->Init(acc_dist, cruising_speed_, 0);
      ADEBUG << "ego cruise motion init, v = " << cruising_speed_ << " ,a = " << 0.0
             << " , end_t = " << start_dec_t;
      piecewise_motions_ptr->emplace_back(start_cruise_t, start_dec_t,
                                          cruise_motion);

      // constant deceleration model
      ConstantAccelerationModelT* dec_motion = new ConstantAccelerationModelT();
      dec_motion->Init(stop_s - dec_dist, cruising_speed_, d_comfort);
      ADEBUG << "ego comfort dec motion init, v = " << cruising_speed_
             << " ,a = " << d_comfort
             << " ,end_t: " << start_dec_t - cruising_speed_ / d_comfort;
      piecewise_motions_ptr->emplace_back(
          start_dec_t, FLAGS_trajectory_time_length, dec_motion);
    }
  }

  return;
}

void Trajectory1dGenerator::set_weights(
    const SpeedDeciderWeights& weights_config) {
  weight_guide_velocity_ = weights_config.guide_velocity();
  weight_lon_accelerate_ = weights_config.accelerate();
  weight_lon_jerk_ = weights_config.jerk();
  weight_collision_dist_ = weights_config.collision_dist();
  weight_follow_dist_ = weights_config.follow_dist();
  weight_centripetal_acc_ = weights_config.centripetal_acc();
}

void Trajectory1dGenerator::CalculateQuinticPolyCurve(
    const double start_s, const double start_v, const double start_a,
    const double end_s, const double end_v, const double end_a, const double dt,
    PolynomialXOrder* const result) {
  double t1_2 = dt * dt;
  double t1_3 = dt * t1_2;
  double t1_4 = t1_2 * t1_2;
  double t1_5 = t1_2 * t1_3;
  double rf = start_s;
  double re = start_v;
  double rd = start_a * 0.5;
  double rc = (20 * (end_s - start_s) - (8 * end_v + 12 * start_v) * dt +
               (end_a - 3 * start_a) * t1_2) /
              2.0 / t1_3;
  double rb = (30 * (start_s - end_s) + (14 * end_v + 16 * start_v) * dt +
               (3 * start_a - 2 * end_a) * t1_2) /
              2.0 / t1_4;
  double ra = (12 * (end_s - start_s) - 6 * (end_v + start_v) * dt +
               (end_a - start_a) * t1_2) /
              2.0 / t1_5;
  result->Init({rf, re, rd, rc, rb, ra});
  result->set_start_state({start_s, start_v, start_a});
  result->set_end_state(
      std::pair<std::array<double, 3>, double>({end_s, end_v, end_a}, dt));
};

void Trajectory1dGenerator::CalculateQuarticPolyCurve(
    const double start_s, const double start_v, const double start_a,
    const double end_v, const double end_a, const double dt,
    PolynomialXOrder* const result) {
  double t1_2 = dt * dt;
  double t1_3 = dt * t1_2;
  double re = start_s;
  double rd = start_v;
  double rc = start_a * 0.5;
  double rb = (3 * (end_v - start_v) - 4 * rc * dt - end_a * dt) / (3 * t1_2);
  double ra = (start_v - end_v + rc * dt + 0.5 * end_a * dt) / (2 * t1_3);
  result->Init({re, rd, rc, rb, ra});
  result->set_start_state({start_s, start_v, start_a});
  result->set_end_state(
      std::pair<std::array<double, 3>, double>({0.0, end_v, end_a}, dt));
};

}  // namespace planning
}  // namespace xju
