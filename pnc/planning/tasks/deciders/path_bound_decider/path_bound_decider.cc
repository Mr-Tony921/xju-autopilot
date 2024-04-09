/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/deciders/path_bound_decider/path_bound_decider.h"

#include <math.h>

#include <deque>

#include "common/logger/logger.h"
#include "common/vehicle_config/vehicle_config_provider.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "planning/common/reference_line_info/reference_line_info.h"

#include "common/util/coordinate.h"
#include "common/vehicle_state/vehicle_state_provider.h"

namespace xju {
namespace planning {
namespace {
// PathBoundPoint contains: (s, l_min, l_max).
using PathBoundPoint = std::tuple<double, double, double>;
// PathBound contains a vector of PathBoundPoint.
using PathBound = std::vector<PathBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id).
using ObstacleEdge = std::tuple<int, double, double, double, std::string>;

}  // namespace

bool is_overlap(const ObstacleBoundSection& section_1,
                const ObstacleBoundSection& section_2) {
  if (section_1.start_idx > section_2.end_idx ||
      section_1.end_idx < section_2.start_idx) {
    return false;
  } else {
    return true;
  }
}

ObstacleBoundSection overlap_section(const ObstacleBoundSection& section_1,
                                     const ObstacleBoundSection& section_2) {
  ObstacleBoundSection section;
  section.start_idx = section_1.end_idx;
  section.end_idx = section_1.start_idx;
  if (section_1.start_idx > section_2.end_idx ||
      section_1.end_idx < section_2.start_idx) {
    return section;
  } else {
    section.start_idx = std::max(section_1.start_idx, section_2.start_idx);
    section.end_idx = std::min(section_1.end_idx, section_2.end_idx);
    return section;
  }
}

bool IsWithinPathDeciderScopeObstacle(const Obstacle& obstacle) {
  // Obstacle should be non-virtual.
  if (obstacle.is_virtual()) {
    ADEBUG << "obstacle(" << obstacle.id() << ") is virtual.";
    return false;
  }
  // Obstacle should not have ignore decision.
  if (obstacle.IsIgnore()) {
    ADEBUG << "obstacle(" << obstacle.id() << ") is ignore.";
    return false;
  }
  // Obstacle should not be moving obstacle.
  if (obstacle.speed() > FLAGS_static_obstacle_speed_threshold) {
    ADEBUG << "obstacle(" << obstacle.id() << ") is not static, and speed is "
          << obstacle.speed();
    return false;
  }
  return true;
}

void maxSlidingWindow(const std::vector<double>& origin_nums,
                      const int window_size,
                      std::vector<double>* const result) {
  std::deque<int> mono_deque;
  if (origin_nums.empty() || result == nullptr) {
    return;
  }
  if (origin_nums.size() < window_size) {
    maxSlidingWindow(origin_nums, origin_nums.size(), result);
    return;
  }
  for (int i = 0; i < origin_nums.size(); ++i) {
    if (!mono_deque.empty() && mono_deque.front() == i - window_size * 2) {
      mono_deque.pop_front();
    }
    while (!mono_deque.empty() &&
           origin_nums[mono_deque.back()] < origin_nums[i]) {
      mono_deque.pop_back();
    }
    mono_deque.push_back(i);
    if (i >= window_size - 1) {
      result->push_back(origin_nums[mono_deque.front()]);
    }
  }
}

void PrintBound(const PathBound& bound) {
  int count = 0;
  for (const auto p : bound) {
    ADEBUG << count++ << ", " << std::get<0>(p) << ", " << std::get<1>(p)
           << ", " << std::get<2>(p);
  }
}

PathBoundDecider::PathBoundDecider(
    const pnc::TaskConfig& config,
    const std::shared_ptr<PlanningInternal>& internal)
    : Task(config, internal) {}

void PathBoundDecider::Init(const pnc::TaskConfig& config) { config_ = config; }

void PathBoundDecider::Reset() {}

bool PathBoundDecider::Process(
    std::shared_ptr<ReferenceLineInfo> const reference_line_info,
    Frame* const frame) {
  PathBound fallback_bound;
  PathBound lane_bound;
  PathBound obstacles_bound;
  std::string blocking_obstacle_id;
  bool is_bound_clear = true;  // There is no obstacles

  // Check Input
  if (frame == nullptr || reference_line_info == nullptr) {
    ADEBUG << " Input is null";
    return false;
  }

  // Initialize some private variables
  InitPathBoundDecider(*frame, *reference_line_info);

  // Initialize the path boundaries to be an indefinitely large area.
  if (!InitLaneBound(*reference_line_info, &lane_bound)) {
    AERROR << "Failed to initialize flane bound.";
    return false;
  }
  // Generate the lane bound(fallback bound) (without obstacle).
  bool ret = GenerateLaneBound(*reference_line_info, &lane_bound);
  if (!ret) {
    AERROR << "Can't generate lane bound.";
    return false;
  }

  if (adc_frenet_l_ < std::get<1>(lane_bound[0]) ||
      adc_frenet_l_ > std::get<2>(lane_bound[0])) {
    AERROR << "adc_frenet_l_ " << adc_frenet_l_ << " "
           << std::get<1>(lane_bound[0]) << " " << std::get<2>(lane_bound[0]);
    AERROR << "Adc is out of lane boundary.";
    return false;
  }

  fallback_bound.insert(fallback_bound.begin(), lane_bound.begin(),
                        lane_bound.end());
  // PrintBound(fallback_bound);
  
  if (!InitObstacleBound(*reference_line_info, lane_bound, &obstacles_bound)) {
    AERROR << "Failed to initialize obstacle bound.";
    return false;
  }

  std::vector<ObstacleBoundSection> left_obstacle_bound_sections;
  std::vector<ObstacleBoundSection> right_obstacle_bound_sections;
  ret = GenerateObstaclesBound(*reference_line_info, &obstacles_bound,
                               &left_obstacle_bound_sections,
                               &right_obstacle_bound_sections);
  ADEBUG << "left_obstacle_bound_sections size : "
         << left_obstacle_bound_sections.size();
  ADEBUG << "right_obstacle_bound_sections size : "
         << right_obstacle_bound_sections.size();

  if (!ret) {
    AERROR << " Failed to generate obstacles bound.";
    return false;
  }

  if (lane_bound.size() != obstacles_bound.size()) {
    AERROR << "Obstacles bound is not equal to lane bound.";
    return false;
  }
  // PrintBound(obstacles_bound);
  std::vector<ObstacleBoundSection> externded_left_obstacle_bound_sections;
  std::vector<ObstacleBoundSection> externded_right_obstacle_bound_sections;

  if (!ExtendBound(left_obstacle_bound_sections, right_obstacle_bound_sections,
                   &externded_left_obstacle_bound_sections,
                   &externded_right_obstacle_bound_sections, &lane_bound,
                   &obstacles_bound)) {
    AERROR << "Externd bound failed.";
    return false;
  }

  ADEBUG << "externded_left_obstacle_bound_sections size "
         << externded_left_obstacle_bound_sections.size();
  ADEBUG << "externded_right_obstacle_bound_sections size "
         << externded_right_obstacle_bound_sections.size();
  AERROR << "left section";
  for (const auto p : externded_left_obstacle_bound_sections) {
    AERROR << p.start_idx << " " << p.end_idx;
  }
  AERROR << "right section";
  for (const auto p : externded_right_obstacle_bound_sections) {
    AERROR << p.start_idx << " " << p.end_idx;
  }
  // PrintBound(lane_bound);
  // PrintBound(obstacles_bound);
  CheckBlocking(*reference_line_info, externded_left_obstacle_bound_sections,
                externded_right_obstacle_bound_sections, &lane_bound,
                &obstacles_bound, &blocking_obstacle_id, &is_bound_clear);
  UpdateLaneBoundByObstacleBound(obstacles_bound, &lane_bound);
  // put the bound into the reference_line_info
  //-------------fallback boundary --------------------------------
  std::vector<std::pair<double, double>> fallback_bound_pair;
  PathBoundary fallback_boundary;
  for (size_t i = 0; i < fallback_bound.size(); ++i) {
    fallback_bound_pair.emplace_back(std::get<1>(fallback_bound[i]),
                                     std::get<2>(fallback_bound[i]));
  }
  fallback_boundary.set_start_s(std::get<0>(fallback_bound[0]));
  fallback_boundary.set_delta_s(
      config_.path_bound_decider_config().s_resolution());
  fallback_boundary.set_lane_boundary(fallback_bound_pair);
  fallback_boundary.set_obstacle_boundary(fallback_bound_pair);
  fallback_boundary.set_path_boundary(fallback_bound_pair);
  fallback_boundary.set_blocking_obstacle_id(blocking_obstacle_id);
  fallback_boundary.set_label("fallback");

  //-------------regular boundary --------------------------------
  std::vector<std::pair<double, double>> lane_bound_pair;
  std::vector<std::pair<double, double>> obstacles_bound_pair;
  std::vector<std::pair<double, double>> path_bound_pair;
  PathBoundary regular_boundary;
  for (size_t i = 0; i < lane_bound.size(); ++i) {
    lane_bound_pair.emplace_back(std::get<1>(lane_bound[i]),
                                 std::get<2>(lane_bound[i]));
  }
  for (size_t i = 0; i < obstacles_bound.size(); ++i) {
    obstacles_bound_pair.emplace_back(std::get<1>(obstacles_bound[i]),
                                      std::get<2>(obstacles_bound[i]));
  }
  GeneratePathBoundary(lane_bound_pair, obstacles_bound_pair, &path_bound_pair);
  
  regular_boundary.set_start_s(std::get<0>(lane_bound[0]));
  regular_boundary.set_delta_s(
      config_.path_bound_decider_config().s_resolution());
  regular_boundary.set_lane_boundary(lane_bound_pair);
  regular_boundary.set_obstacle_boundary(obstacles_bound_pair);
  regular_boundary.set_blocking_obstacle_id(blocking_obstacle_id);
  regular_boundary.set_path_boundary(path_bound_pair);
  regular_boundary.set_label("regular");
  regular_boundary.set_is_path_boundary_clear(is_bound_clear);  // TODO
  //-------------path boundary------------------------
  reference_line_info->SetBlockingObstacle(blocking_obstacle_id);
  auto* path_boundary = reference_line_info->mutable_path_boundary();
  path_boundary->push_back(regular_boundary);
  path_boundary->push_back(fallback_boundary);
  //-------------show boundary in rviz------------------------
  DrawDebugInfo(regular_boundary, "regular_boundary");
  DrawDebugInfo(fallback_boundary, "fallback_boundary");

  return true;
}

void PathBoundDecider::InitPathBoundDecider(
    const Frame& frame, const ReferenceLineInfo& reference_line_info) {
  const pnc_map::ReferenceLine& reference_line =
      reference_line_info.reference_line();
  pnc::TrajectoryPoint planning_start_point = frame.planning_start_point();

  auto adc_sl_info = reference_line.ToFrenetFrame(planning_start_point);
  adc_frenet_s_ = adc_sl_info.first.at(0);
  adc_frenet_l_ = adc_sl_info.second.at(0);
  adc_frenet_sd_ = adc_sl_info.first.at(1);
  adc_frenet_ld_ = adc_sl_info.second.at(1) * adc_frenet_sd_;
  ADEBUG << " adc_frenet_s_ = " << adc_frenet_s_
         << " adc_frenet_l_ = " << adc_frenet_l_
         << " adc_frenet_sd_ = " << adc_frenet_sd_
         << " adc_frenet_ld_ = " << adc_frenet_ld_;

  const auto& vehicle_config = pnc::VehicleConfigProvider::GetConfig();
  adc_length_ = vehicle_config.length();
  adc_width_ = vehicle_config.width();

  const auto& task_config = config_.path_bound_decider_config();
  s_range_start_ = 0.0;
  s_range_end_ =
      std::fmin(reference_line.length(),
                adc_frenet_s_ + std::fmax(task_config.s_horizon(),
                                          reference_line.max_speed_limit() *
                                              FLAGS_trajectory_time_length));
  s_resolution_ = task_config.s_resolution();
  ADEBUG << "s range (" << s_range_start_ << "," << s_range_end_ << ")";
  CalculateTrajectoryKappas(reference_line, s_range_start_, s_range_end_,
                            s_resolution_, &filted_kappas_);
}

void PathBoundDecider::CalculateTrajectoryKappas(
    const pnc_map::ReferenceLine& reference_line, const double s_range_start,
    const double s_range_end, const double s_resolution,
    std::vector<double>* const reference_points_kappas) {
  // record kappas in s_range and filt by sliding window
  reference_points_kappas->clear();

  const auto myInterpolation = [](const double s1, const double s2,
                                  const double value1, const double value2,
                                  double s) -> double {
    double ds = s2 - s1;
    double dvalue = value2 - value1;
    if (ds < pnc::kMathEpsilon) {
      return value1;
    } else {
      return dvalue * (s - s1) / ds + value1;
    }
  };

  // int ref_count = 0;
  // for (const auto& p : reference_line.reference_points()) {
  //   AERROR << "ref kappa No." << ref_count++ << " " << p.kappa();
  // }

  size_t i = 0;
  std::vector<double> origin_reference_points_kappas;

  for (double curr_s = s_range_start; curr_s < s_range_end;
       curr_s += s_resolution) {
    while (reference_line.reference_points().at(i).s() <
           curr_s - pnc::kMathEpsilon) {
      ++i;
    }
    if (i == 0) {
      origin_reference_points_kappas.push_back(
          std::fabs(reference_line.reference_points().at(i).kappa()));
    } else {
      double kappa = myInterpolation(
          reference_line.reference_points().at(i - 1).s(),
          reference_line.reference_points().at(i).s(),
          reference_line.reference_points().at(i - 1).kappa(),
          reference_line.reference_points().at(i).kappa(), curr_s);
      origin_reference_points_kappas.push_back(std::fabs(kappa));
    }
  }

  FiltKappas(origin_reference_points_kappas, reference_points_kappas);
  // AERROR << " origin_reference_points_kappas size "
  //        << origin_reference_points_kappas.size();
  // AERROR << " reference_points_kappas size " << reference_points_kappas->size();
  // ADEBUG << "Origin kappas >>> ";
  // for (const auto& kappa : origin_reference_points_kappas) {
  //   ADEBUG << kappa << " , ";
  // }
  // ADEBUG << "<<< ";

  // ADEBUG << "Filted kappas >>> ";
  // for (const auto& kappa : *reference_points_kappas) {
  //   ADEBUG << kappa << " ";
  // }
  // ADEBUG << "<<< ";
}

bool PathBoundDecider::InitLaneBound(
    const ReferenceLineInfo& reference_line_info, PathBound* const lane_bound) {
  // Sanity checks.
  if (lane_bound == nullptr) {
    AERROR << "Path Bound is nullptr.";
    return false;
  }

  lane_bound->clear();

  const auto& reference_line = reference_line_info.reference_line();
  // Starting from ADC's current position, increment until the horizon, and
  // set lateral bounds to be infinite at every spot.
  for (double curr_s = s_range_start_; curr_s < s_range_end_;
       curr_s += s_resolution_) {
    lane_bound->emplace_back(curr_s, std::numeric_limits<double>::lowest(),
                             std::numeric_limits<double>::max());
  }

  if (lane_bound->empty()) {
    AERROR << "Empty lane boundary in InitLaneBound.";
    return false;
  }
  ADEBUG << "Init lane bound size is " << lane_bound->size();
  return true;
}

bool PathBoundDecider::GenerateLaneBound(
    const ReferenceLineInfo& reference_line_info, PathBound* const lane_bound) {
  ADEBUG << "-------Calculate lane bound processing------";
  // Decide a rough boundary based on lane info and ADC's position
  if (!GetBoundaryFromLanesAndADC(reference_line_info, lane_bound)) {
    AERROR << "Failed to decide a rough fallback boundary based on "
              "road information.";
    return false;
  }
  if (lane_bound->empty()) {
    AERROR << "Failed to get a valid lane boundary.";
    return false;
  }
  ADEBUG << "Completed generating lane bound. Lane bound size is "
        << lane_bound->size();
  return true;
}

bool PathBoundDecider::InitObstacleBound(
    const ReferenceLineInfo& reference_line_info, const PathBound& lane_bound,
    PathBound* const obstacles_bound) {
  ADEBUG << "-------Init obstacle bound------";
  if (obstacles_bound == nullptr) {
    AERROR << "Obstacle Bound is nullptr.";
    return false;
  }
  obstacles_bound->clear();

  constexpr double kBoundaryBuffer = 0.05;  // meter
  bool is_left_lane_boundary = true;
  bool is_right_lane_boundary = true;
  double curr_road_left_width = 0.0;
  double curr_road_right_width = 0.0;
  double curr_s = std::get<0>(lane_bound.at(0));
  double curr_lane_right_width = std::get<1>(lane_bound.at(0));
  double curr_lane_left_width = std::get<2>(lane_bound.at(0));

  if (reference_line_info.reference_line().GetRoadWidth(
          curr_s, curr_road_left_width, curr_road_right_width)) {
    is_left_lane_boundary = std::fabs(curr_road_left_width -
                                      curr_lane_left_width) > kBoundaryBuffer;
    is_right_lane_boundary = std::fabs(curr_road_right_width -
                                       curr_lane_right_width) > kBoundaryBuffer;
  }

  ADEBUG << "is_left_lane_boundary ? " << is_left_lane_boundary
         << ", is_right_lane_boundary ? " << is_right_lane_boundary;
  ADEBUG << "left_lane_bound_soft_ ? " << left_lane_bound_soft_
         << ", right_lane_bound_soft_ ? " << right_lane_bound_soft_;

  double kBoundBuffLeft =
      (is_left_lane_boundary && left_lane_bound_soft_) ? 1.0 : 0.0;
  double kBoundBuffRight =
      (is_right_lane_boundary && right_lane_bound_soft_) ? 1.0 : 0.0;
  ADEBUG << "kBoundBuffLeft : " << kBoundBuffLeft
         << " kBoundBuffRight : " << kBoundBuffRight;
  for (const auto& bound : lane_bound) {
    obstacles_bound->emplace_back(std::get<0>(bound),
                                  std::get<1>(bound) - kBoundBuffRight,
                                  std::get<2>(bound) + kBoundBuffLeft);
  }
  if (obstacles_bound->empty()) {
    AERROR << "Empty obstalce boundary in InitObstacleBound.";
    return false;
  }
  if (obstacles_bound->size() != lane_bound.size()) {
    AERROR << "Obstacles bound is not equal to lane bound size.";
    return false;
  }
  return true;
}

bool PathBoundDecider::GenerateObstaclesBound(
    const ReferenceLineInfo& reference_line_info,
    PathBound* const obstacles_bound,
    std::vector<ObstacleBoundSection>* const left_obstacle_bound_sections,
    std::vector<ObstacleBoundSection>* const right_obstacle_bound_sections) {
  if (obstacles_bound->empty()) {
    AERROR << "obstacles_bound is empty.";
    return false;
  }
  // sort obstacles
  auto obstacles = reference_line_info.path_decision().obstacles();
  auto sorted_obstacles = SortObstaclesForSweepLine(obstacles);

  double center_line = adc_frenet_l_;
  size_t obs_idx = 0;

  std::multiset<double, std::greater<double>> right_bounds;  // large -> small
  std::multiset<double> left_bounds;
  right_bounds.insert(std::numeric_limits<double>::lowest());
  left_bounds.insert(std::numeric_limits<double>::max());

  std::map<size_t, std::string> left_section_start_idxs;
  std::map<size_t, std::string> right_section_start_idxs;

  left_obstacle_bound_sections->clear();
  right_obstacle_bound_sections->clear();
  // Maps obstacle ID's to the decided ADC pass direction, if ADC should
  // pass from left, then true; otherwise, false.
  std::unordered_map<std::string, bool> obs_id_to_direction;

  // Step through every path point.
  for (size_t i = 1; i < obstacles_bound->size(); ++i) {
    double curr_s = std::get<0>((*obstacles_bound)[i]);
    // Check and see if there is any obstacle change:
    if (obs_idx < sorted_obstacles.size() &&
        std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
      while (obs_idx < sorted_obstacles.size() &&
             std::get<1>(sorted_obstacles[obs_idx]) < curr_s) {
        const auto& curr_obstacle = sorted_obstacles[obs_idx];
        const double curr_obstacle_s = std::get<1>(curr_obstacle);
        const double curr_obstacle_l_min = std::get<2>(curr_obstacle);
        const double curr_obstacle_l_max = std::get<3>(curr_obstacle);
        const std::string curr_obstacle_id = std::get<4>(curr_obstacle);
        ADEBUG << "id[" << curr_obstacle_id << "] s[" << curr_obstacle_s
              << "] curr_obstacle_l_min[" << curr_obstacle_l_min
              << "] curr_obstacle_l_max[" << curr_obstacle_l_max
              << "] center_line[" << center_line << "]";
        if (std::get<0>(curr_obstacle) == 1) {
          if (curr_obstacle_l_min + curr_obstacle_l_max <
              center_line * 2 + pnc::kMathEpsilon) {
            // Obstacle is to the right of center-line, should pass from left.
            // ADEBUG << "Obstacle is to the right of center-line, should pass from left";
            obs_id_to_direction[curr_obstacle_id] = true;
            right_bounds.insert(curr_obstacle_l_max);
            right_section_start_idxs.insert({i, curr_obstacle_id});
          } else {
            // Obstacle is to the left of center-line, should pass from right.
            // ADEBUG << "Obstacle is to the left of center-line, should pass from right";
            obs_id_to_direction[curr_obstacle_id] = false;
            left_bounds.insert(curr_obstacle_l_min);
            left_section_start_idxs.insert({i, curr_obstacle_id});
          }
          UpdateObstaclesBoundaryAndCenterLineWithBuffer(
              i, *left_bounds.begin(), *right_bounds.begin(), obstacles_bound,
              &center_line);
        } else {
          // An existing obstacle exits our scope.
          if (obs_id_to_direction[curr_obstacle_id]) {
            right_bounds.erase(right_bounds.find(curr_obstacle_l_max));
            for (auto iter = right_section_start_idxs.begin();
                 iter != right_section_start_idxs.end(); ++iter) {
              if (iter->second == curr_obstacle_id) {
                right_obstacle_bound_sections->emplace_back(iter->first, i,
                                                            curr_obstacle_id);
                right_section_start_idxs.erase(iter->first);
                break;
              }
            }
          } else {
            left_bounds.erase(left_bounds.find(curr_obstacle_l_min));
            for (auto iter = left_section_start_idxs.begin();
                 iter != left_section_start_idxs.end(); ++iter) {
              if (iter->second == curr_obstacle_id) {
                left_obstacle_bound_sections->emplace_back(iter->first, i,
                                                           curr_obstacle_id);
                left_section_start_idxs.erase(iter->first);
                break;
              }
            }
          }
          obs_id_to_direction.erase(curr_obstacle_id);
        }
        // Update the bounds and center_line.
        std::get<1>((*obstacles_bound)[i]) = std::fmax(
            std::get<1>((*obstacles_bound)[i]), *right_bounds.begin());
        std::get<2>((*obstacles_bound)[i]) =
            std::fmin(std::get<2>((*obstacles_bound)[i]), *left_bounds.begin());
        center_line = (std::get<1>((*obstacles_bound)[i]) +
                       std::get<2>((*obstacles_bound)[i])) / 2.0;
        ++obs_idx;
      }
    } else {
      // If no obstacle change, update the bounds and center_line.
      std::get<1>((*obstacles_bound)[i]) =
          std::fmax(std::get<1>((*obstacles_bound)[i]), *right_bounds.begin());
      std::get<2>((*obstacles_bound)[i]) =
          std::fmin(std::get<2>((*obstacles_bound)[i]), *left_bounds.begin());
      center_line = (std::get<1>((*obstacles_bound)[i]) +
                     std::get<2>((*obstacles_bound)[i])) / 2.0;
    }
  }

  std::sort(left_obstacle_bound_sections->begin(),
            left_obstacle_bound_sections->end(),
            [](const ObstacleBoundSection& s1, const ObstacleBoundSection& s2) {
              if (s1.start_idx != s2.start_idx) {
                return s1.start_idx < s2.start_idx;
              } else {
                return s1.end_idx < s2.end_idx;
              }
            });
  std::sort(right_obstacle_bound_sections->begin(),
            right_obstacle_bound_sections->end(),
            [](const ObstacleBoundSection& s1, const ObstacleBoundSection& s2) {
              if (s1.start_idx != s2.start_idx) {
                return s1.start_idx < s2.start_idx;
              } else {
                return s1.end_idx < s2.end_idx;
              }
            });
  if (obstacles_bound->empty()) {
    AERROR << "Failed to get a valid obstacles boundary.";
    return false;
  }

  return true;
}

bool PathBoundDecider::CheckBlocking(
    const ReferenceLineInfo& reference_line_info,
    const std::vector<ObstacleBoundSection>&
        externded_left_obstacle_bound_sections,
    const std::vector<ObstacleBoundSection>&
        externded_right_obstacle_bound_sections,
    PathBound* const lane_bound, PathBound* const obstacles_bound,
    std::string* const blocking_obstacle_id, bool* const is_bound_clear) const {
  if (lane_bound == nullptr || obstacles_bound == nullptr) {
    AERROR << "Input bound is null.";
    return false;
  }
  if (lane_bound->size() != obstacles_bound->size()) {
    AERROR << "Lane bound size is not equal to obstacles bound size.";
    return false;
  }

  int path_blocked_idx = -1;

  double adc_length_base =
      pnc::VehicleConfigProvider::GetConfig().wheel_base();
  double adc_length = adc_length_;
  ADEBUG << "adc_length_base " << adc_length_base << " adc_length "
         << adc_length << " adc_width_ " << adc_width_;
  for (size_t i = 0; i < lane_bound->size(); ++i) {
    double lane_width_limited_by_kappa = GetWidthAccordingKappa(
        adc_length_base, adc_width_, filted_kappas_.at(i), 0.0);
    double obtacle_width_limited_by_kappa = GetWidthAccordingKappa(
        adc_length, adc_width_, filted_kappas_.at(i), 0.0);
    if (std::get<2>(lane_bound->at(i)) - std::get<1>(lane_bound->at(i)) <
            lane_width_limited_by_kappa ||
        std::get<2>(obstacles_bound->at(i)) -
                std::get<1>(obstacles_bound->at(i)) <
            obtacle_width_limited_by_kappa) {
      path_blocked_idx = i;
      AERROR << "path_blocked_idx: " << path_blocked_idx
             << ", lane_width_limited_by_kappa: " << lane_width_limited_by_kappa
             << ", obtacle_width_limited_by_kappa : "
             << obtacle_width_limited_by_kappa << " filted_kappas_.at(i) "
             << filted_kappas_.at(i);
      break;
    }
  }

  double d = adc_length_ * 2.0;
  double k = -d / adc_width_;
  const auto lon_distance_limit = [&](const double lat_dis) -> double {
    double lat_dis_clamped = std::clamp(
        lat_dis, -config_.path_bound_decider_config().default_lane_width(),
        config_.path_bound_decider_config().default_lane_width());
    double lon = std::fmax(0.0, k * lat_dis_clamped + d);
    return lon;
  };

  const auto opposite_back_section = [](const auto& begin, const auto& end,
                                        const auto& value) {
    for (auto it = begin; it != end; ++it) {
      if (it->end_idx < value) {
        return it;
      }
    }
    return end;
  };

  auto opposite_back_section_iter =
      externded_right_obstacle_bound_sections.rend();
  for (auto left_iter = externded_left_obstacle_bound_sections.begin();
       left_iter != externded_left_obstacle_bound_sections.end(); ++left_iter) {
    opposite_back_section_iter =
        opposite_back_section(externded_right_obstacle_bound_sections.rbegin(),
                              opposite_back_section_iter, left_iter->start_idx);
    if (opposite_back_section_iter ==
        externded_right_obstacle_bound_sections.rend()) {
      AERROR << "rend";
      continue;
    } else {
      double lat_dis =
          std::get<2>(obstacles_bound->at((left_iter->start_idx))) -
          std::get<1>(
              obstacles_bound->at((opposite_back_section_iter->start_idx)));
      double lon_dis =
          s_resolution_ * static_cast<int>(left_iter->start_idx -
                                           opposite_back_section_iter->end_idx);
      AERROR << "left_iter start_idx : " << left_iter->start_idx;
      AERROR << "opposite_back_section_iter end_idx : "
             << opposite_back_section_iter->end_idx;
      AERROR << "lon_dis " << lon_dis << ", lat_dis " << lat_dis
             << ", lon_distance_limit(lat_dis) " << lon_distance_limit(lat_dis);
      if (lon_dis < lon_distance_limit(lat_dis)) {
        path_blocked_idx = left_iter->start_idx;
        AERROR << "path_blocked_idx: " << path_blocked_idx;
        break;
      }
    }
  }

  opposite_back_section_iter = externded_left_obstacle_bound_sections.rend();
  for (auto right_iter = externded_right_obstacle_bound_sections.begin();
       right_iter != externded_right_obstacle_bound_sections.end();
       ++right_iter) {
    if (path_blocked_idx > 0 && right_iter->end_idx > path_blocked_idx) {
      break;
    }
    opposite_back_section_iter = opposite_back_section(
        externded_left_obstacle_bound_sections.rbegin(),
        opposite_back_section_iter, right_iter->start_idx);
    if (opposite_back_section_iter ==
        externded_left_obstacle_bound_sections.rend()) {
      AERROR << "rend";
      continue;
    } else {
      double lat_dis =
          std::get<2>(
              obstacles_bound->at((opposite_back_section_iter->start_idx))) -
          std::get<1>(obstacles_bound->at((right_iter->start_idx)));

      double lon_dis =
          s_resolution_ * static_cast<int>(right_iter->start_idx -
                                           opposite_back_section_iter->end_idx);
      AERROR << "right_iter start_idx : " << right_iter->start_idx;
      AERROR << "opposite_back_section_iter end_idx : "
             << opposite_back_section_iter->end_idx;
      AERROR << "lon_dis " << lon_dis << ", lat_dis " << lat_dis
             << ", lon_distance_limit(lat_dis) " << lon_distance_limit(lat_dis);
      if (lon_dis < lon_distance_limit(lat_dis)) {
        path_blocked_idx = right_iter->start_idx;
        AERROR << "path_blocked_idx: " << path_blocked_idx;
        break;
      }
    }
  }
  if (!externded_left_obstacle_bound_sections.empty()) {
    if (path_blocked_idx >
        externded_left_obstacle_bound_sections.front().start_idx) {
      *is_bound_clear = false;
    }
  }
  if (!externded_right_obstacle_bound_sections.empty()) {
    if (path_blocked_idx >
        externded_right_obstacle_bound_sections.front().start_idx) {
      *is_bound_clear = false;
    }
  }

  std::string left_blocking_obstacle_id = "";
  std::string right_blocking_obstacle_id = "";
  int left_blocked_idx = std::numeric_limits<int>::max();
  int right_blocked_idx = std::numeric_limits<int>::max();
  for (const auto& section : externded_left_obstacle_bound_sections) {
    if (section.start_idx <= path_blocked_idx && section.end_idx >= path_blocked_idx) {
      left_blocking_obstacle_id = section.obstacle_id;
      left_blocked_idx = section.start_idx;
      break;
    }
  }

  for (const auto& section : externded_right_obstacle_bound_sections) {
    if (section.start_idx <= path_blocked_idx && section.end_idx >= path_blocked_idx) {
      right_blocking_obstacle_id = section.obstacle_id;
      right_blocked_idx = section.start_idx;
      break;
    }
  }
  *blocking_obstacle_id = left_blocked_idx < right_blocked_idx
                             ? left_blocking_obstacle_id
                             : right_blocking_obstacle_id;

  ADEBUG << " CheckBlocking , path_blocked_idx is " << path_blocked_idx;
  ADEBUG << " CheckBlocking , blocking_obstacle_id is " << *blocking_obstacle_id;
  TrimPathBounds(path_blocked_idx, lane_bound);
  TrimPathBounds(path_blocked_idx, obstacles_bound);
  return true;
}

bool PathBoundDecider::UpdateLaneBoundByObstacleBound(
    const PathBound& obstacles_bound, PathBound* const lane_bound) {
  if (lane_bound == nullptr) {
    AERROR << "lane_bound is null.";
    return false;
  }
  if (lane_bound->size() != obstacles_bound.size()) {
    AERROR << "lane_bound size is not equal to obstacles_bound size.";
    return false;
  }

  for (size_t i = 0; i < obstacles_bound.size(); ++i) {
    std::get<1>(lane_bound->at(i)) = std::fmax(
        std::get<1>(lane_bound->at(i)), std::get<1>(obstacles_bound.at(i)));
    std::get<2>(lane_bound->at(i)) = std::fmin(
        std::get<2>(lane_bound->at(i)), std::get<2>(obstacles_bound.at(i)));
  }
  return true;
}

bool PathBoundDecider::ExtendBound(
    const std::vector<ObstacleBoundSection>& left_obstacle_bound_sections,
    const std::vector<ObstacleBoundSection>& right_obstacle_bound_sections,
    std::vector<ObstacleBoundSection>* const
        externded_left_obstacle_bound_sections,
    std::vector<ObstacleBoundSection>* const
        externded_right_obstacle_bound_sections,
    PathBound* const lane_bound, PathBound* const obstacles_bound) const {
  // 1. Input check.
  if (lane_bound == nullptr || obstacles_bound == nullptr) {
    AERROR << "Input bound is invalid.";
    return false;
  }
  if (externded_left_obstacle_bound_sections == nullptr ||
      externded_right_obstacle_bound_sections == nullptr) {
    AERROR << "Input is invalid.";
    return false;
  }

  if (left_obstacle_bound_sections.empty() &&
      right_obstacle_bound_sections.empty()) {
    ADEBUG << "There is no obstacle sections.";
    return true;
  }

  std::vector<ObstacleBoundSection> single_left_obstalce_sections;
  std::vector<ObstacleBoundSection> single_right_obstalce_sections;

  for (const auto& left_obstacle_bound_section : left_obstacle_bound_sections) {
    std::vector<ObstacleBoundSection>
        overlap_sections;  // right sections that overlap with left_section
    double right_l =
        std::numeric_limits<double>::lowest();  // overlap_sections' l max
    for (const auto& right_obstacle_bound_section :
         right_obstacle_bound_sections) {
      const auto& overlap = overlap_section(left_obstacle_bound_section,
                                            right_obstacle_bound_section);
      if (overlap.start_idx <= overlap.end_idx) {
        overlap_sections.push_back(right_obstacle_bound_section);
        if (right_l < std::get<1>(obstacles_bound->at(overlap.start_idx))) {
          right_l = std::get<1>(obstacles_bound->at(overlap.start_idx));
        }
      }
      if (right_obstacle_bound_section.start_idx >
          left_obstacle_bound_section.end_idx) {
        break;
      }
    }

    if (overlap_sections.empty()) {
      single_left_obstalce_sections.emplace_back(left_obstacle_bound_section);
      externded_left_obstacle_bound_sections->emplace_back(
          left_obstacle_bound_section);
    } else {
      ADEBUG << "Overlap sections size is " << overlap_sections.size();
      /* obstacles are on both sides, and extend obstacle bounds in longtitude
      —————————.        .   .—————————————————————————————————————
      obstacle_bound
      ---------|________|>>>|--------------------------------------- lane_bound

                     _______
      ---------|<<<<|-------|---------------------------------------
      —————————.    .       .————————————————————————————————————————
            extend obstacle bound
          |<---------------->|

      for (auto p : overlap_sections) {
        ADEBUG << " Overlap section(" << p.start_idx << " " << p.end_idx << ") ";
      }
      ADEBUG << left_obstacle_bound_section.start_idx << " " <<
      left_obstacle_bound_section.end_idx;
      */

      const size_t start_extend_idx =
          std::min(left_obstacle_bound_section.start_idx,
                   overlap_sections.front().start_idx);
      const size_t end_extend_idx = std::max(
          left_obstacle_bound_section.end_idx, overlap_sections.back().end_idx);
      // extend right obstacle bound at overlaps sections
      for (size_t i = start_extend_idx; i < end_extend_idx; ++i) {
        std::get<1>(obstacles_bound->at(i)) =
            std::fmax(std::get<1>(obstacles_bound->at(i)), right_l);
      }
      // extend left section in longitudinal
      for (size_t i = left_obstacle_bound_section.start_idx;
           i >= start_extend_idx; --i) {
        std::get<2>(obstacles_bound->at(i)) = std::fmin(
            std::get<2>(obstacles_bound->at(i)),
            std::get<2>(
                obstacles_bound->at(left_obstacle_bound_section.start_idx)));
      }
      for (size_t i = left_obstacle_bound_section.end_idx; i <= end_extend_idx;
           ++i) {
        std::get<2>(obstacles_bound->at(i)) = std::fmin(
            std::get<2>(obstacles_bound->at(i)),
            std::get<2>(
                obstacles_bound->at(left_obstacle_bound_section.start_idx)));
      }

      if (!externded_left_obstacle_bound_sections->empty() &&
          is_overlap(
              externded_left_obstacle_bound_sections->back(),
              ObstacleBoundSection(start_extend_idx, end_extend_idx, ""))) {
        ADEBUG << "left ovelap";
        externded_left_obstacle_bound_sections->back().end_idx =
            std::max(externded_left_obstacle_bound_sections->back().end_idx,
                     end_extend_idx);
      } else {
        ADEBUG << "left no ovelap";
        externded_left_obstacle_bound_sections->emplace_back(
            start_extend_idx, end_extend_idx,
            left_obstacle_bound_section.obstacle_id);
      }

      if (!externded_right_obstacle_bound_sections->empty() &&
          is_overlap(
              externded_right_obstacle_bound_sections->back(),
              ObstacleBoundSection(start_extend_idx, end_extend_idx, ""))) {
        ADEBUG << "right ovelap";
        externded_right_obstacle_bound_sections->back().end_idx =
            std::max(externded_right_obstacle_bound_sections->back().end_idx,
                     end_extend_idx);
      } else {
        ADEBUG << "right no ovelap";
        externded_right_obstacle_bound_sections->emplace_back(
            start_extend_idx, end_extend_idx,
            overlap_sections.front().obstacle_id);
      }
    }
  }

  for (const auto& right_obstacle_bound_section :
       right_obstacle_bound_sections) {
    std::vector<ObstacleBoundSection> overlap_sections;
    for (const auto& left_obstacle_bound_section :
         left_obstacle_bound_sections) {
      // const auto& overlap = overlap_section(right_obstacle_bound_section,
      // left_obstacle_bound_section);
      if (is_overlap(left_obstacle_bound_section,
                     right_obstacle_bound_section)) {
        overlap_sections.push_back(left_obstacle_bound_section);
      }
      if (left_obstacle_bound_section.start_idx >
          right_obstacle_bound_section.end_idx) {
        break;
      }
    }
    if (overlap_sections.empty()) {
      single_right_obstalce_sections.emplace_back(right_obstacle_bound_section);
      externded_right_obstacle_bound_sections->emplace_back(
          right_obstacle_bound_section);
    }
  }

  std::sort(externded_right_obstacle_bound_sections->begin(),
            externded_right_obstacle_bound_sections->end(),
            [](const ObstacleBoundSection& s1, const ObstacleBoundSection& s2) {
              return s1.start_idx < s2.start_idx;
            });

  /* obstacles are on left side, and extend right lane bound
  —————————.     .————————————————————————————————————————— obstacle_bound
  ---------|_____|------------------------------------------ lane_bound
               |
  ----.        V         .---------------------------------
  ————.——————————————————.—————————————————————————————————
        extend lane bound
      |<---------------->|
  */

  ExtendLaneBound(single_left_obstalce_sections, true, *obstacles_bound,
                  lane_bound);
  ExtendLaneBound(single_right_obstalce_sections, false, *obstacles_bound,
                  lane_bound);
  return true;
}

void PathBoundDecider::ExtendLaneBound(
    const std::vector<ObstacleBoundSection>& sections, const bool is_left,
    const PathBound& obstacles_bound, PathBound* const lane_bound) const {
  constexpr double kLonExtendDistance = 10.0;  // m
  auto origin_lane_bound = *lane_bound;
  for (const auto& section : sections) {
    size_t start_extend_idx = std::max(
        static_cast<int>(section.start_idx) -
            static_cast<int>(std::ceil(kLonExtendDistance / s_resolution_)),
        0);
    size_t end_extend_idx = std::min(
        static_cast<int>(section.end_idx) +
            static_cast<int>(std::ceil(kLonExtendDistance / s_resolution_)),
        static_cast<int>(origin_lane_bound.size() - 1));
    if (is_left) {  // externd right lane bound
      double ext_l = std::fmax(
          0.0, std::get<2>(origin_lane_bound.at(section.start_idx)) -
                   std::get<2>(obstacles_bound.at(section.start_idx)));
      for (size_t i = start_extend_idx; i < end_extend_idx; ++i) {
        std::get<1>(lane_bound->at(i)) =
            std::fmin(std::get<1>(origin_lane_bound.at(i)) - ext_l,
                      std::get<1>(lane_bound->at(i)));
        std::get<1>(lane_bound->at(i)) = std::fmax(
            std::get<1>(lane_bound->at(i)), std::get<1>(obstacles_bound.at(i)));
      }
    } else {  // externd left lane bound
      double ext_l = std::fmax(
          0.0, std::get<1>(obstacles_bound.at(section.start_idx)) -
                   std::get<1>(origin_lane_bound.at(section.start_idx)));
      for (size_t i = start_extend_idx; i < end_extend_idx; ++i) {
        std::get<2>(lane_bound->at(i)) =
            std::fmax(std::get<2>(origin_lane_bound.at(i)) + ext_l,
                      std::get<2>(lane_bound->at(i)));
        std::get<2>(lane_bound->at(i)) = std::fmin(
            std::get<2>(lane_bound->at(i)), std::get<2>(obstacles_bound.at(i)));
      }
    }
  }
};

bool PathBoundDecider::GetBoundaryFromLanesAndADC(
    const ReferenceLineInfo& reference_line_info, PathBound* const lane_bound) {
  // Sanity checks.
  if (lane_bound == nullptr || lane_bound->empty()) {
    return false;
  }
  const pnc_map::ReferenceLine& reference_line =
      reference_line_info.reference_line();

  left_lane_bound_soft_ = true;
  right_lane_bound_soft_ = true;
  double shift_distance_to_left = 0.0;
  double shift_distance_to_right = 0.0;
  if (internal_->planning_status().lateral_shift_status().type() ==
      LateralShiftStatus::LEFT) {
    shift_distance_to_left =
        internal_->planning_status().lateral_shift_status().distance_m();
    left_lane_bound_soft_ = false;
    ADEBUG << "left lane_bound type is hard, shift_distance_to_right is "
           << shift_distance_to_right;
  } else if (internal_->planning_status().lateral_shift_status().type() ==
             LateralShiftStatus::RIGHT) {
    shift_distance_to_right =
        internal_->planning_status().lateral_shift_status().distance_m();
    right_lane_bound_soft_ = false;
    ADEBUG << "right lane_bound type is hard, shift_distance_to_left is "
           << shift_distance_to_left;
  }

  // cancle shift if kappa is larger than set value
  if (internal_->planning_status().lateral_shift_status().type() !=
      LateralShiftStatus::NONE) {
    constexpr double kKappaThresholdToCancleShift = 0.005;
    constexpr int kKappaCheckTime = 3;
    const double kappa_check_distance =
        kKappaCheckTime * reference_line.max_speed_limit();
    double kappa_max = 0.0;
    for (int i = 0;
         i * s_resolution_ < kKappaCheckTime * kappa_check_distance &&
         i < filted_kappas_.size();
         ++i) {
      if (filted_kappas_.at(i) > kKappaThresholdToCancleShift) {
        ADEBUG << "Kappas(" << filted_kappas_.at(i)
               << " is larger than threshold(" << kKappaThresholdToCancleShift
               << ").";
        shift_distance_to_left = 0.0;
        shift_distance_to_right = 0.0;
        break;
      }
    }
  }
  // Go through every point, update the boundary based on lane info and
  // ADC's position.
  double past_lane_left_width =
      config_.path_bound_decider_config().default_lane_width() / 2.0;
  double past_lane_right_width =
      config_.path_bound_decider_config().default_lane_width() / 2.0;

  double curr_left_bound_adc = 0.0;
  double curr_right_bound_adc = 0.0;

  ADCBoundWithBuffer(reference_line_info, &curr_left_bound_adc,
                     &curr_right_bound_adc);

  double adc_length =
      pnc::VehicleConfigProvider::GetConfig().wheel_base();

  // if adc bound is greater than lane bound, then the lane is not soft bound,
  // and obstacle bound is the same as lane bound
  ADEBUG << "curr_left_bound_adc = " << curr_left_bound_adc
         << ", curr_right_bound_adc = " << curr_right_bound_adc;

  int path_blocked_idx = -1;

  // constexpr double kWidthMargin = 0.5;
  for (size_t i = 0; i < lane_bound->size(); ++i) {
    double curr_s = std::get<0>((*lane_bound)[i]);
    // 1. Get the current lane width at current point.
    double curr_lane_left_width = 0.0;
    double curr_lane_right_width = 0.0;
    if (!reference_line.GetLaneWidth(curr_s, curr_lane_left_width,
                                     curr_lane_right_width)) {
      AERROR << "Failed to get lane width at s = " << curr_s;
      curr_lane_left_width = past_lane_left_width;
      curr_lane_right_width = past_lane_right_width;
    }

    past_lane_left_width = curr_lane_left_width;
    past_lane_right_width = curr_lane_right_width;

    // 2. Calculate the proper boundary based on shift distance, ADC's position,
    //    and ADC's velocity.
    double curr_left_bound_lane = curr_lane_left_width - shift_distance_to_left;
    double curr_right_bound_lane =
        -curr_lane_right_width + shift_distance_to_right;

    // extend path bounds to include ADC .
    double curr_left_bound =
        std::fmax(curr_left_bound_lane, curr_left_bound_adc);
    double curr_right_bound =
        std::fmin(curr_right_bound_lane, curr_right_bound_adc);

    // 3. Update the boundary by adc bound.
    double width_limited_by_kappa = GetWidthAccordingKappa(
        adc_length, adc_width_, filted_kappas_.at(i),
        config_.path_bound_decider_config().adc_bound_buffer() * 2);
    // ADEBUG << "width_limited_by_kappa " << width_limited_by_kappa;

    if (!UpdatePathBoundWithKappa(i, curr_left_bound, curr_right_bound,
                                  width_limited_by_kappa, lane_bound)) {
      AERROR << "UpdatePathBoundWithKappa failed. Blocked at " << i;
      path_blocked_idx = static_cast<int>(i);
    }
    if (path_blocked_idx != -1) {
      ADEBUG << "path blocked index is : " << path_blocked_idx;
      break;
    }
  }
  ADEBUG << "lane_bound size = " << lane_bound->size();
  TrimPathBounds(path_blocked_idx, lane_bound);
  ADEBUG << "Trim lane_bound size = " << lane_bound->size();
  ADEBUG << "final bound is " << std::get<0>((*lane_bound).front()) << " : "
         << std::get<1>((*lane_bound).front()) << " ~ "
         << std::get<2>((*lane_bound).front());
  return true;
}

void PathBoundDecider::ADCBoundWithBuffer(
    const ReferenceLineInfo& reference_line_info, double* adc_bound_left,
    double* adc_bound_right) {
  *adc_bound_left = reference_line_info.car_sl_boundary().end_l();
  *adc_bound_right = reference_line_info.car_sl_boundary().start_l();
  ADEBUG << "adc box bound " << *adc_bound_left << " " << *adc_bound_right;

  ADEBUG << "left " << reference_line_info.car_sl_boundary().end_l();
  ADEBUG << "right " << reference_line_info.car_sl_boundary().start_l();

  double adc_speed_buffer =
      (adc_frenet_ld_ > 0 ? 1.0 : -1.0) * adc_frenet_ld_ * adc_frenet_ld_ /
      config_.path_bound_decider_config().max_lateral_acceleration() / 2.0;
  ADEBUG << "adc_speed_buffer " << adc_speed_buffer;

  *adc_bound_left =
      std::fmax(*adc_bound_left, *adc_bound_left + adc_speed_buffer) +
      config_.path_bound_decider_config().adc_bound_buffer();
  *adc_bound_right =
      std::fmin(*adc_bound_right, *adc_bound_right + adc_speed_buffer) -
      config_.path_bound_decider_config().adc_bound_buffer();
  ADEBUG << "adc bound " << *adc_bound_left << " " << *adc_bound_right;
}

bool PathBoundDecider::UpdatePathBoundWithKappa(
    const size_t idx, const double left_bound, const double right_bound,
    const double width_limited_by_kappa, PathBound* const lane_bound) const {
  // Update the right bound (l_min):
  double new_l_min = std::fmax(std::get<1>((*lane_bound)[idx]), right_bound);
  // Update the left bound (l_max):
  double new_l_max = std::fmin(std::get<2>((*lane_bound)[idx]), left_bound);

  if (new_l_max - new_l_min < width_limited_by_kappa) {
    if (left_lane_bound_soft_ && right_lane_bound_soft_) {
      // ADEBUG << " externd bound to both sides." << width_limited_by_kappa;
      double d_l = new_l_max - new_l_min;
      new_l_max += (width_limited_by_kappa - d_l) * 0.5;
      new_l_min -= (width_limited_by_kappa - d_l) * 0.5;

    } else if (left_lane_bound_soft_) {
      // ADEBUG << " externd bound to left sides." << width_limited_by_kappa;
      new_l_max += width_limited_by_kappa - (new_l_max - new_l_min);
    } else if (right_lane_bound_soft_) {
      // ADEBUG << " externd bound to right sides." << width_limited_by_kappa;
      new_l_min -= width_limited_by_kappa - (new_l_max - new_l_min);
    } else {
      ADEBUG << "Block index is " << idx;
      return false;
    }
  }
  // Otherwise, update lane bound and center_line; then return true.
  std::get<1>((*lane_bound)[idx]) = new_l_min;
  std::get<2>((*lane_bound)[idx]) = new_l_max;
  // ADEBUG << "idx " << idx << " nee_l_min " << new_l_min << " new_l_max "
  //       << new_l_max << "  width_limited_by_kapa " << width_limited_by_kappa;
  return true;
}

double PathBoundDecider::GetBufferBetweenADCCenterAndEdge() {
  double adc_half_width = adc_width_ / 2.0;

  return (adc_half_width +
          config_.path_bound_decider_config().adc_edge_buffer());
}

void PathBoundDecider::TrimPathBounds(const int path_blocked_idx,
                                      PathBound* const path_boundaries) const {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      ADEBUG << "Completely blocked. Cannot move at all";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

// The tuple contains (is_start_s, s, l_min, l_max, obstacle_id)
std::vector<ObstacleEdge> PathBoundDecider::SortObstaclesForSweepLine(
    const IndexedList<std::string, Obstacle>& indexed_obstacles) {
  std::vector<ObstacleEdge> sorted_obstacles;
  // Go through every obstacle and preprocess it.
  for (const auto* obstacle : indexed_obstacles.Items()) {
    // Only focus on those within-scope obstacles.
    if (!IsWithinPathDeciderScopeObstacle(*obstacle)) {
      continue;
    }
    // Only focus on obstacles that are ahead of ADC.
    if (obstacle->sl_boundary().end_s() < adc_frenet_s_) {
      ADEBUG << "obstacle(" << obstacle->id() << ") is behind ADC.";
      continue;
    }

    // Decompose each obstacle's rectangle into two edges: one at
    // start_s; the other at end_s.
    const auto obstacle_sl = obstacle->sl_boundary();
    ADEBUG << "obstacle info :";
    ADEBUG << "id = " << obstacle->id() << ", s(" << obstacle_sl.start_s()
           << "," << obstacle_sl.end_s() << ")"
           << ", l(" << obstacle_sl.start_l() << "," << obstacle_sl.end_l()
           << ")"
           << "(x, y, heading) " << obstacle->perception_obstacle().position().x()
           << ", " << obstacle->perception_obstacle().position().y() 
           << ", " << obstacle->perception_obstacle().heading()<< ")";
    ADEBUG << "FLAGS_obstacle_lon_start_buffer "
           << FLAGS_obstacle_lon_start_buffer;
    ADEBUG << "FLAGS_obstacle_lon_end_buffer " << FLAGS_obstacle_lon_end_buffer;
    ADEBUG << "FLAGS_obstacle_lat_buffer " << FLAGS_obstacle_lat_buffer;
    sorted_obstacles.emplace_back(
        1, obstacle_sl.start_s() - FLAGS_obstacle_lon_start_buffer,
        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->id());
    sorted_obstacles.emplace_back(
        0, obstacle_sl.end_s() + FLAGS_obstacle_lon_end_buffer,
        obstacle_sl.start_l() - FLAGS_obstacle_lat_buffer,
        obstacle_sl.end_l() + FLAGS_obstacle_lat_buffer, obstacle->id());
  }

  // Sort.
  std::sort(sorted_obstacles.begin(), sorted_obstacles.end(),
            [](const ObstacleEdge& lhs, const ObstacleEdge& rhs) {
              if (std::get<1>(lhs) != std::get<1>(rhs)) {
                return std::get<1>(lhs) < std::get<1>(rhs);
              } else {
                return std::get<0>(lhs) > std::get<0>(rhs);
              }
            });
  for (size_t i = 0; i < sorted_obstacles.size(); ++i) {
    ADEBUG << "sorted_obstacles is ";
    ADEBUG << "No." << i << " index is " << std::get<4>(sorted_obstacles[i])
           << " is_start? " << std::get<0>(sorted_obstacles[i]) << " s is "
           << std::get<1>(sorted_obstacles[i]) << " l is "
           << std::get<2>(sorted_obstacles[i]);
  }
  return sorted_obstacles;
}

void PathBoundDecider::UpdateObstaclesBoundaryAndCenterLineWithBuffer(
    size_t idx, double left_bound, double right_bound,
    PathBound* const path_boundaries, double* const center_line) {
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]), right_bound);
  double new_l_max =
      std::fmin(std::get<2>((*path_boundaries)[idx]), left_bound);

  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;

  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2.0;
}

bool PathBoundDecider::GeneratePathBoundary(
    const std::vector<std::pair<double, double>>& lane_bound_pair,
    const std::vector<std::pair<double, double>>& obstacles_bound_pair,
    std::vector<std::pair<double, double>>* const path_bound_pair) {
  if (lane_bound_pair.size() != obstacles_bound_pair.size()) {
    return false;
  }
  size_t bound_size = lane_bound_pair.size();
  for (size_t i = 0; i < bound_size; ++i) {
    double l_min =
        std::fmax(lane_bound_pair[i].first, obstacles_bound_pair[i].first);
    double l_max =
        std::fmin(lane_bound_pair[i].second, obstacles_bound_pair[i].second);
    path_bound_pair->emplace_back(l_min, l_max);
  }
  return true;
}

/* post process*/
double PathBoundDecider::GetWidthAccordingKappa(const double length,
                                                const double width,
                                                const double kappa,
                                                const double margin) const {
  double width_limit = width + margin;
  if (std::fabs(kappa) < 1e-6) {
    return width_limit;
  }
  const double radius = std::fabs(1.0 / kappa);
  width_limit = std::sqrt((length * length) +
                          (radius + width / 2.0) * (radius + width / 2.0)) -
                radius + width / 2.0 + margin;

  return width_limit;
}

void PathBoundDecider::FiltKappas(
    const std::vector<double>& origin_kappas,
    std::vector<double>* const filted_kappas) const {
  constexpr double kBackWindow = 15.0;   // m
  constexpr double kFrontWindow = 15.0;  // m
  if (origin_kappas.empty() || filted_kappas == nullptr) return;

  filted_kappas->clear();

  const int window_size = (kBackWindow + kFrontWindow) / s_resolution_;
  maxSlidingWindow(origin_kappas, window_size, filted_kappas);

  for (std::size_t i = filted_kappas->size(); i < origin_kappas.size(); ++i) {
    filted_kappas->push_back(filted_kappas->back());
  }
}

void PathBoundDecider::DrawDebugInfo(const PathBoundary& path_bound,
                                     const std::string& title) {
  constexpr double x_min = -10.0;
  constexpr double x_max = 250.0;
  constexpr double y_min = -5.0;
  constexpr double y_max = 5.0;
  pnc::Debug* debug_info = internal_->mutable_debug_info();
  pnc::Chart* chart_bound = debug_info->add_charts();
  chart_bound->set_title(title);
  chart_bound->mutable_options()->mutable_x()->set_min(x_min);
  chart_bound->mutable_options()->mutable_x()->set_max(x_max);
  chart_bound->mutable_options()->mutable_x()->set_label_string("s");

  chart_bound->mutable_options()->mutable_y()->set_min(y_min);
  chart_bound->mutable_options()->mutable_y()->set_max(y_max);
  chart_bound->mutable_options()->mutable_y()->set_label_string("l");

  auto ego_polygon = chart_bound->add_polygon();
  auto left_back = ego_polygon->add_point();
  auto right_back = ego_polygon->add_point();
  auto right_front = ego_polygon->add_point();
  auto left_front = ego_polygon->add_point();

  left_back->set_x(0);
  left_back->set_y(adc_frenet_l_ + adc_width_ / 2);

  left_front->set_x(0 + adc_length_);
  left_front->set_y(adc_frenet_l_ + adc_width_ / 2);

  right_back->set_x(0);
  right_back->set_y(adc_frenet_l_ - adc_width_ / 2);

  right_front->set_x(0 + adc_length_);
  right_front->set_y(adc_frenet_l_ - adc_width_ / 2);

  const double s_start = path_bound.start_s() - adc_frenet_s_;
  const double ds = path_bound.delta_s();
  double s = s_start;

  auto lane_left_bound_line = chart_bound->add_line();
  auto lane_right_bound_line = chart_bound->add_line();
  lane_left_bound_line->set_label("LaneBound_L");
  (*lane_left_bound_line->mutable_properties())["color"] = "g";

  lane_right_bound_line->set_label("LaneBound_R");
  (*lane_right_bound_line->mutable_properties())["color"] = "g";

  for (const auto& bound : path_bound.lane_boundary()) {
    pnc::Point2D* p_l = lane_left_bound_line->add_point();
    pnc::Point2D* p_r = lane_right_bound_line->add_point();

    p_l->set_x(s);
    p_l->set_y(bound.first);
    p_r->set_x(s);
    p_r->set_y(bound.second);

    s += ds;
  }

  for (int i = -3.5; i < 3.5; i += 3.5) {
    auto center_line = chart_bound->add_line();
    (*center_line->mutable_properties())["color"] = "r";
    pnc::Point2D* start_p = center_line->add_point();
    pnc::Point2D* end_p = center_line->add_point();
    start_p->set_x(x_min);
    start_p->set_y(i);
    end_p->set_x(x_max);
    end_p->set_y(i);
  }

  s = s_start;

  auto obstacle_left_bound_line = chart_bound->add_line();
  auto obstacle_right_bound_line = chart_bound->add_line();
  obstacle_left_bound_line->set_label("ObjBound_L");
  (*obstacle_left_bound_line->mutable_properties())["color"] = "y";
  obstacle_right_bound_line->set_label("ObjBound_R");
  (*obstacle_right_bound_line->mutable_properties())["color"] = "y";

  for (const auto& bound : path_bound.obstacle_boundary()) {
    pnc::Point2D* p_l = obstacle_left_bound_line->add_point();
    pnc::Point2D* p_r = obstacle_right_bound_line->add_point();

    p_l->set_x(s);
    p_l->set_y(bound.first);
    p_r->set_x(s);
    p_r->set_y(bound.second);

    s += ds;
  }
}

}  // namespace planning
}  // namespace xju
