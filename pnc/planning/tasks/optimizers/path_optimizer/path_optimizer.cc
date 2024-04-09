/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/path_optimizer/path_optimizer.h"

#include <ctime>
#include <fstream>
#include <future>

#include "common/logger/logger.h"
#include "common/util/vehicle_helper.h"

namespace xju {
namespace planning {

namespace {
using xju::pnc::PathPoint;
using xju::pnc::TrajectoryPoint;
using xju::pnc_map::ReferenceLine;
} // namespace

PathOptimizer::PathOptimizer(
    const pnc::TaskConfig& config, 
    const std::shared_ptr<PlanningInternal>& internal)
    : Task(config, internal) {
  vehicle_model_optimizer_.Init();
  if (config_.path_optimizer_config().enable_multi_thread()) {
    for (auto& optimizer : vehicle_model_optimizers_) {
      optimizer.Init();
    }
  }
}

void PathOptimizer::Init(const pnc::TaskConfig& config) {
  config_ = config;
  vehicle_model_optimizer_.Init();
}

void PathOptimizer::Reset() {
  
}

bool PathOptimizer::Process(
    std::shared_ptr<ReferenceLineInfo> const reference_line_info, Frame* const frame) {
  Task::Process(reference_line_info, frame);

  if (!config_.path_optimizer_config().enabled()) {
    AERROR << "Skip PathOptimizer according config.";
    return true;
  }

  if (!internal_->planning_status().has_lane_change_status()) {
    AERROR << "Do not get lane change status.";
    return false;
  }

  if (config_.path_optimizer_config().enable_multi_thread()) {
    ADEBUG << "optimizer with multi thread.";
    return OptimizeMultiThread();
  } else {
    ADEBUG << "optimizer with single thread.";
    return OptimizeSequence();
  }
}

bool PathOptimizer::OptimizeMultiThread() {
  const auto& planning_start_point = 
      reference_line_info_->planning_start_point().path_point();

  const auto& lane_change_status = internal_->planning_status().lane_change_status();

  PathOptimizerWeights optimizer_weights = config_.path_optimizer_config().defalut_weights();
  if (lane_change_status.status() == LaneChangeStatus::LANE_CHANGE) {
    ADEBUG << "IN LaneChange!";
    optimizer_weights = config_.path_optimizer_config().lane_change_weights();
  }
  
  double init_speed = reference_line_info_->planning_start_point().v();
  double cruise_speed = reference_line_info_->cruise_speed();
  
  const auto& reference_line = reference_line_info_->reference_line();

  const std::vector<PathBoundary>& path_boundarys = reference_line_info_->path_boundary();
  if (path_boundarys.size() != 2) {
    AERROR << "Path Boundary size is not equal to 2!";
    return false;
  }
  
  std::vector<std::future<bool>> results;
  for (int i = 0; i < 2; ++i) {
    auto& optimizer = vehicle_model_optimizers_[i];
    optimizer.set_weights(optimizer_weights);
    optimizer.SetCarSpeedInfo(init_speed, cruise_speed);
    if (internal_->planning_status().has_path_point_to_pass_through()) {
      auto& target = internal_->planning_status().path_point_to_pass_through();
      optimizer.SetTarget(target.x(), target.y(), target.theta());
    }
    const auto& boundary = path_boundarys[i];
    results.push_back(
        std::async(std::launch::async, &VehicleModelOptimizer::OptimizePath, &optimizer, 
            reference_line, planning_start_point, &boundary));
  }
  
  bool status = false;
  for (auto& result : results) {
    status = result.get() || status;
  }

  if (!status) {
    AERROR << "Path Optimizer failed with all boundary.";
    return false;
  }

  bool use_regular = false;
  for (auto& optimizer : vehicle_model_optimizers_) {
    if (optimizer.status() && optimizer.boundary_label() == "regular") {
      use_regular = true;
    }
  }

  auto* path_data = reference_line_info_->mutable_path_data();
  for (auto& optimizer : vehicle_model_optimizers_) {
    if (use_regular && optimizer.boundary_label() == "regular") {
      optimizer.GetOptimizePath(path_data);
      break;
    } else if (!use_regular && optimizer.boundary_label() == "fallback") {
      optimizer.GetOptimizePath(path_data);
      break;
    } 
  }

  DrawSLDebugInfo();
  DrawCartesianDebugInfo();
  // LogPlannedPath();
  return true;
}

bool PathOptimizer::OptimizeSequence() {
  const auto& planning_start_point = 
      reference_line_info_->planning_start_point().path_point();

  const auto& lane_change_status = internal_->planning_status().lane_change_status();
  if (lane_change_status.status() == LaneChangeStatus::LANE_CHANGE) {
    ADEBUG << "IN LaneChange!";
    vehicle_model_optimizer_.set_weights(config_.path_optimizer_config().lane_change_weights());
  } else {
    ADEBUG << "IN LaneFollow!";
    vehicle_model_optimizer_.set_weights(config_.path_optimizer_config().defalut_weights());
  }
  if (internal_->planning_status().has_path_point_to_pass_through()) {
    auto& target = internal_->planning_status().path_point_to_pass_through();
    vehicle_model_optimizer_.SetTarget(target.x(), target.y(), target.theta());
  }

  double init_speed = reference_line_info_->planning_start_point().v();
  double cruise_speed = reference_line_info_->cruise_speed();
  vehicle_model_optimizer_.SetCarSpeedInfo(init_speed, cruise_speed);
  
  const auto& reference_line = reference_line_info_->reference_line();

  const std::vector<PathBoundary>& path_boundarys = reference_line_info_->path_boundary();
  bool status = false;
  for (const auto& path_boundary : path_boundarys) {
    if (!path_boundary.Vailed()) {
      AERROR << path_boundary.label() << "path boundary is not vailed!";
      continue;
    }
    
    status = vehicle_model_optimizer_.OptimizePath(
        reference_line, planning_start_point, &path_boundary);
    if (!status) {
      AERROR << "Vehicle Model Optimizer Failed with " << path_boundary.label() << " boundary.";
      // LogInfoToFile();
      continue;
    }

    auto* path_data = reference_line_info_->mutable_path_data();
    status = vehicle_model_optimizer_.GetOptimizePath(path_data);
    
    if (status && path_boundary.label() == "regular") {
      break;
    }
    
  }

  if (!status) {
    AERROR << "Can Not get optimize path.";
    return false;
  }
  
  reference_line_info_->AddCost(std::fabs(vehicle_model_optimizer_.cost()));
  DrawSLDebugInfo();
  DrawCartesianDebugInfo();
  // LogPlannedPath();
  return true;
}

void PathOptimizer::DrawSLDebugInfo() {
  pnc::Debug* debug_info = internal_->mutable_debug_info();

  const auto& reference_points = 
      reference_line_info_->reference_line().reference_points();

  const auto& path_data = reference_line_info_->path_data();
  const auto& planned_sl_path = path_data.frenet_points();
  const auto& planned_path = path_data.planned_path();

  const auto& path_boundarys = reference_line_info_->path_boundary();
  const auto path_boundary = path_boundarys.front();
  double start_s = path_boundary.start_s();
  double delta_s = path_boundary.delta_s();

  // add s-l chart
  pnc::Chart* chart_s_l = debug_info->add_charts();
  chart_s_l->set_title("s-l");
  chart_s_l->mutable_options()->mutable_x()->set_min(0);
  chart_s_l->mutable_options()->mutable_x()->set_max(150);
  chart_s_l->mutable_options()->mutable_x()->set_label_string("s(m)");

  chart_s_l->mutable_options()->mutable_y()->set_min(-6);
  chart_s_l->mutable_options()->mutable_y()->set_max(6);
  chart_s_l->mutable_options()->mutable_y()->set_label_string("l(m)");

  pnc::Line* line_l = chart_s_l->add_line();
  line_l->set_label("OCP");
  (*line_l->mutable_properties())["color"] = "k";
  for (const auto& sl_pt : planned_sl_path) {
    pnc::Point2D* point2d = line_l->add_point();
    point2d->set_x(sl_pt.s());
    point2d->set_y(sl_pt.l());
  }

  pnc::Line* line_l_lower_lane = chart_s_l->add_line();
  line_l_lower_lane->set_label("lane lower bound");
  (*line_l_lower_lane->mutable_properties())["color"] = "b";
  pnc::Line* line_l_upper_lane = chart_s_l->add_line();
  line_l_upper_lane->set_label("lane upper bound");
  (*line_l_upper_lane->mutable_properties())["color"] = "b";
  for (int i = 0; i < path_boundary.lane_boundary().size(); ++i) {
    const auto& bound = path_boundary.lane_boundary()[i];
    pnc::Point2D* lower = line_l_lower_lane->add_point();
    lower->set_x(start_s + delta_s * i);
    lower->set_y(bound.first);

    pnc::Point2D* upper = line_l_upper_lane->add_point();
    upper->set_x(start_s + delta_s * i);
    upper->set_y(bound.second);
  }

  pnc::Line* line_l_lower_obstacle = chart_s_l->add_line();
  line_l_lower_obstacle->set_label("obstacle lower bound");
  (*line_l_lower_obstacle->mutable_properties())["color"] = "r";
  pnc::Line* line_l_upper_obstacle = chart_s_l->add_line();
  line_l_upper_obstacle->set_label("obstacle upper bound");
  (*line_l_upper_obstacle->mutable_properties())["color"] = "r";
  for (int i = 0; i < path_boundary.obstacle_boundary().size(); ++i) {
    const auto& bound = path_boundary.obstacle_boundary()[i];
    pnc::Point2D* lower = line_l_lower_obstacle->add_point();
    lower->set_x(start_s + delta_s * i);
    lower->set_y(bound.first);

    pnc::Point2D* upper = line_l_upper_obstacle->add_point();
    upper->set_x(start_s + delta_s * i);
    upper->set_y(bound.second);
  }

  double theta_min = 100.0;
  double theta_max = -100.0;
  double kappa_min = 100.0;
  double kappa_max = -100.0;
  for (const auto& pt : reference_points) {
    theta_min = std::fmin(theta_min, pt.theta());
    theta_max = std::fmax(theta_max, pt.theta());
    kappa_min = std::fmin(kappa_min, pt.kappa());
    kappa_max = std::fmax(kappa_max, pt.kappa());
  }

  for (const auto& pt : planned_path) {
    theta_min = std::fmin(theta_min, pt.theta());
    theta_max = std::fmax(theta_max, pt.theta());
    kappa_min = std::fmin(kappa_min, pt.kappa());
    kappa_max = std::fmax(kappa_max, pt.kappa());
  }

  // add s-theta chart
  pnc::Chart* chart_theta = debug_info->add_charts();
  chart_theta->set_title("s-theta");
  chart_theta->mutable_options()->mutable_x()->set_min(0);
  chart_theta->mutable_options()->mutable_x()->set_max(150);
  chart_theta->mutable_options()->mutable_x()->set_label_string("s(m)");

  chart_theta->mutable_options()->mutable_y()->set_min(theta_min);
  chart_theta->mutable_options()->mutable_y()->set_max(theta_max);
  chart_theta->mutable_options()->mutable_y()->set_label_string("theta(rad)");
  pnc::Line* line_theta = chart_theta->add_line();
  
  line_theta->set_label("theta");
  (*line_theta->mutable_properties())["color"] = "k";
  for (int i = 0; i < planned_path.size(); ++i) {
    const auto& pt = planned_path[i];
    const auto& sl_pt = planned_sl_path[i];
    pnc::Point2D* point2d = line_theta->add_point();
    point2d->set_x(sl_pt.s());
    point2d->set_y(pt.theta());
  }

  pnc::Line* line_theta_ref = chart_theta->add_line();
  line_theta_ref->set_label("ref theta");
  (*line_theta_ref->mutable_properties())["color"] = "r";
  for (const auto& pt : reference_points) {
    pnc::Point2D* point2d = line_theta_ref->add_point();
    point2d->set_x(pt.s());
    point2d->set_y(pt.theta());
  }

  // add s-kappa chart
  pnc::Chart* chart_kappa = debug_info->add_charts();
  chart_kappa->set_title("s-kappa");
  chart_kappa->mutable_options()->mutable_x()->set_min(0);
  chart_kappa->mutable_options()->mutable_x()->set_max(150);
  chart_kappa->mutable_options()->mutable_x()->set_label_string("s(m)");

  chart_kappa->mutable_options()->mutable_y()->set_min(kappa_min);
  chart_kappa->mutable_options()->mutable_y()->set_max(kappa_max);
  chart_kappa->mutable_options()->mutable_y()->set_label_string("kappa(1/m)");

  pnc::Line* line_kappa = chart_kappa->add_line();
  line_kappa->set_label("kappa");
  (*line_kappa->mutable_properties())["color"] = "k";
  for (const auto& sl_pt : planned_sl_path) {
    pnc::Point2D* point2d = line_kappa->add_point();
    point2d->set_x(sl_pt.s());
    point2d->set_y(sl_pt.kappa());
  }

  pnc::Line* line_kappa_ref = chart_kappa->add_line();
  line_kappa_ref->set_label("ref kappa");
  (*line_kappa_ref->mutable_properties())["color"] = "r";
  for (const auto& pt : reference_points) {
    pnc::Point2D* point2d = line_kappa_ref->add_point();
    point2d->set_x(pt.s());
    point2d->set_y(pt.kappa());
  }

  // pnc::Line* line_kappa_error = chart_kappa->add_line();
  // line_kappa_error->set_label("kappa error");
  // (*line_kappa_error->mutable_properties())["color"] = "r";
  // for (int i = 0; i < planned_sl_path.size(); ++i) {
  //   const auto& sl_pt = planned_sl_path[i];
  //   const auto& pt = path_data.reference_points()[i];
  //   pnc::Point2D* point2d = line_kappa_error->add_point();
  //   point2d->set_x(pt.s());
  //   point2d->set_y(sl_pt.kappa() - pt.kappa());
  // }

  // pnc::Line* line_kappa_ref_final = chart_kappa->add_line();
  // line_kappa_ref_final->set_label("final ref kappa");
  // (*line_kappa_ref->mutable_properties())["color"] = "b";
  // for (const auto& pt : path_data.reference_points()) {
  //   pnc::Point2D* point2d = line_kappa_ref_final->add_point();
  //   point2d->set_x(pt.s());
  //   point2d->set_y(pt.kappa());
  // }

}

void PathOptimizer::DrawCartesianDebugInfo() {
  pnc::Debug* debug_info = internal_->mutable_debug_info();

  const auto& reference_points = 
      reference_line_info_->reference_line().reference_points();

  const auto& path_data = reference_line_info_->path_data();
  const auto& planned_path = path_data.planned_path();

  const auto& path_boundarys = reference_line_info_->path_boundary();
  const auto path_boundary = path_boundarys.front();
  double start_s = path_boundary.start_s();
  double delta_s = path_boundary.delta_s();

  double x_min = 200.0;
  double x_max = -200.0;
  double y_min = 100.0;
  double y_max = -100.0;
  for (const auto& pt : reference_points) {
    x_min = std::fmin(x_min, pt.x());
    x_max = std::fmax(x_max, pt.x());
    y_min = std::fmin(y_min, pt.y());
    y_max =std::fmax(y_max, pt.y());
  }

  // add x-y chart
  pnc::Chart* chart_x_y = debug_info->add_charts();
  chart_x_y->set_title("x-y");
  chart_x_y->mutable_options()->mutable_x()->set_min(x_min - 10.0);
  chart_x_y->mutable_options()->mutable_x()->set_max(x_max + 10.0);
  chart_x_y->mutable_options()->mutable_x()->set_label_string("x(m)");

  chart_x_y->mutable_options()->mutable_y()->set_min(y_min - 6.0);
  chart_x_y->mutable_options()->mutable_y()->set_max(y_max + 6.0);
  chart_x_y->mutable_options()->mutable_y()->set_label_string("l(m)");

  // center line
  pnc::Line* line_ref = chart_x_y->add_line();
  line_ref->set_label("center line");
  (*line_ref->mutable_properties())["color"] = "r";
  for (const auto& pt : reference_points) {
    pnc::Point2D* point2d = line_ref->add_point();
    point2d->set_x(pt.x());
    point2d->set_y(pt.y());
  }
 
  // planned path
  pnc::Line* line_planned = chart_x_y->add_line();
  line_planned->set_label("planned path");
  (*line_planned->mutable_properties())["color"] = "k";
  for (const auto& pt : planned_path) {
    pnc::Point2D* point2d = line_planned->add_point();
    point2d->set_x(pt.x());
    point2d->set_y(pt.y());
  }
  
  pnc::SLPoint sl_pt;
  pnc::Vec2d xy;
  // lane boundary
  pnc::Line* line_lane_bound_l = chart_x_y->add_line();
  line_lane_bound_l->set_label("lane bound");
  (*line_lane_bound_l->mutable_properties())["color"] = "g";

  pnc::Line* line_lane_bound_u = chart_x_y->add_line();
  // line_lane_bound_u->set_label("lane bound");
  (*line_lane_bound_u->mutable_properties())["color"] = "g";
  int i = 0;
  for (const auto& bound : path_boundary.lane_boundary()) {
    sl_pt.set_s(start_s + i * delta_s);
    sl_pt.set_l(bound.first);
    reference_line_info_->reference_line().SLToXY(sl_pt, &xy);
    pnc::Point2D* point2d_l = line_lane_bound_l->add_point();
    point2d_l->set_x(xy.x());
    point2d_l->set_y(xy.y());
    
    sl_pt.set_l(bound.second);
    reference_line_info_->reference_line().SLToXY(sl_pt, &xy);
    pnc::Point2D* point2d_u = line_lane_bound_u->add_point();
    point2d_u->set_x(xy.x());
    point2d_u->set_y(xy.y());
    ++i;
  }

  // obstacle boundary
  pnc::Line* line_obstacle_bound_l = chart_x_y->add_line();
  line_obstacle_bound_l->set_label("obstacle bound");
  (*line_obstacle_bound_l->mutable_properties())["color"] = "b";

  pnc::Line* line_obstacle_bound_u = chart_x_y->add_line();
  // line_obstacle_bound_u->set_label("lane bound");
  (*line_obstacle_bound_u->mutable_properties())["color"] = "b";
  int j = 0;
  for (const auto& bound : path_boundary.obstacle_boundary()) {
    sl_pt.set_s(start_s + j * delta_s);
    sl_pt.set_l(bound.first);
    reference_line_info_->reference_line().SLToXY(sl_pt, &xy);
    pnc::Point2D* point2d_l = line_obstacle_bound_l->add_point();
    point2d_l->set_x(xy.x());
    point2d_l->set_y(xy.y());
    
    sl_pt.set_l(bound.second);
    reference_line_info_->reference_line().SLToXY(sl_pt, &xy);
    pnc::Point2D* point2d_u = line_obstacle_bound_u->add_point();
    point2d_u->set_x(xy.x());
    point2d_u->set_y(xy.y());
    ++j;
  }

  // add vehicle
  double last_s = -100.0;
  const auto& vehicle_config = pnc::VehicleConfigProvider::GetConfig();
  double car_length = vehicle_config.length() + 0.5;
  for (const auto& pt : planned_path) {
    if (std::fabs(pt.s() - last_s) > car_length) {
      pnc::Polygon* car_box = chart_x_y->add_polygon();
      pnc::Box2d box = pnc::VehicleHelper::CarBox(pt);
      for (const auto& corner : box.corners()) {
        pnc::Point2D* point2d = car_box->add_point();
        point2d->set_x(corner.x());
        point2d->set_y(corner.y());
      }
      last_s = pt.s();
    }
  }
  
}

void PathOptimizer::LogPlannedPath() {
  const auto& path_data = reference_line_info_->path_data();
  const auto& planned_sl_path = path_data.frenet_points();
  const auto& planned_path = path_data.planned_path();
  for (int i = 0; i < planned_path.size(); ++i) {
    const auto& sl_pt = planned_sl_path[i];
    const auto& pt = planned_path[i];
    AINFO << sl_pt.ShortDebugString();
    // AINFO << pt.ShortDebugString();
  }
}

void PathOptimizer::CreateLogFile(std::fstream& log_file, std::string name) {
  time_t raw_time;
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);

  std::string dir = getenv("HOME");
  dir += "/path_optimizer_log";

  if (access(dir.c_str(), 0) == -1) {
    std::string command = "mkdir -p " + dir;
    system(command.c_str());
  }

  char time_str[80];
  strftime(time_str, 80, "%F-%H%M%S_", &time_tm);
  dir = dir + "/" + name + time_str + ".csv";
  // dir = dir + "/" + name + ".csv";

  log_file.close();
  log_file.open(dir.c_str(), std::ios_base::out);
  log_file.flags(std::ios::fixed);
}

void PathOptimizer::LogInfoToFile() {

  std::fstream path_data;
  CreateLogFile(path_data, "path_data");
  
  const auto& start_point = reference_line_info_->planning_start_point().path_point();
  path_data << "start point" << std::endl;
  path_data << "x,y,theta,kappa,init_v,target_v" << std::endl;
  path_data << start_point.x() << ","
            << start_point.y() << ","
            << start_point.theta() << ","
            << start_point.kappa() << ","
            << reference_line_info_->planning_start_point().v() << ","
            << reference_line_info_->cruise_speed() << std::endl;

  path_data << "reference line" << std::endl;
  path_data << "x,y,theta,kappa,s" << std::endl;
  const auto& reference_points = reference_line_info_->reference_line().reference_points();
  for (const auto& pt : reference_points) {
    path_data << pt.x() << ","
              << pt.y() << ","
              << pt.theta() << ","
              << pt.kappa() << "," 
              << pt.s() << std::endl;
  }
  
  const auto& boundarys = reference_line_info_->path_boundary();
  const auto boundary = boundarys.front();
  const auto& lane_boundary = boundary.lane_boundary();
  const auto& obstacle_boundary = boundary.obstacle_boundary();
  const auto& path_boundary = boundary.path_boundary();

  path_data << "boundary" << std::endl;
  path_data << "start_s,delta_s" << std::endl;
  path_data << boundary.start_s() << ","
            << boundary.delta_s() << std::endl;
  
  path_data << "lane boundary" << std::endl;
  for (const auto& bound : lane_boundary) {
    path_data << bound.first << "," << bound.second << std::endl;
  }

  path_data << "obstacle boundary" << std::endl;
  for (const auto& bound : obstacle_boundary) {
    path_data << bound.first << "," << bound.second << std::endl;
  }

  path_data << "path boundary" << std::endl;
  for (const auto& bound : path_boundary) {
    path_data << bound.first << "," << bound.second << std::endl;
  }

  path_data.close();
}

} // namespace planning
} // namespace xju
