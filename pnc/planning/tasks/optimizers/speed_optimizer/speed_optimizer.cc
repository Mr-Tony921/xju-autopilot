/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/tasks/optimizers/speed_optimizer/speed_optimizer.h"

#include <google/protobuf/text_format.h>

#include <ctime>
#include <fstream>

#include "planning/common/path/path_data.h"
#include "planning/common/planning_gflags/planning_gflags.h"
#include "planning/common/speed/speed_data.h"
#include "planning/common/speed/st_graph_data.h"
#include "planning/tasks/optimizers/speed_optimizer/piecewise_jerk_speed_optimizer.h"

namespace xju {
namespace planning {
constexpr double kMathEpsilon = 1e-5;

constexpr double kStopBuffer = 5.0;
constexpr double kYieldBuffer = 5.0;
constexpr double kFollowBuffer = 5.0;
constexpr double kOverTakeBuffer = 10.0;

SpeedOptimizer::SpeedOptimizer(
    const pnc::TaskConfig& config,
    const std::shared_ptr<PlanningInternal>& internal)
    : Task(config, internal) {}

void SpeedOptimizer::Init(const pnc::TaskConfig& config) { config_ = config; }

void SpeedOptimizer::Reset() {}

bool SpeedOptimizer::Process(std::shared_ptr<ReferenceLineInfo> const reference_line_info,
                             Frame* const frame) {
  Task::Process(reference_line_info, frame);

  if (!config_.speed_optimizer_config().enabled()) {
    AINFO << "Skip task: " << name() << " by config.";
    return true;
  }

  const SpeedData& reference_speed_data = reference_line_info->speed_data();
  SpeedData* speed_data = reference_line_info->mutable_speed_data();
  const PathData& path_data = reference_line_info->path_data();
  if (path_data.planned_path().empty()) {
    AERROR << "Empty path data";
    return false;
  }
  if (reference_speed_data.empty()) {
    AERROR << "Empty speed data";
    return false;
  }
  // ADEBUG << DebugString();
  // ADEBUG << path_data.DebugString();
  // ADEBUG << reference_speed_data.DebugString();

  const StGraphData& st_graph_data = reference_line_info->st_graph_data();
  const std::array<double, 3> init_s = {0.0, st_graph_data.init_point().v(),
                                        st_graph_data.init_point().a()};
  ADEBUG << "init state: " << init_s[0] << ", " << init_s[1] << ", "
         << init_s[2];

  constexpr double delta_t = 0.1;
  const double total_time = FLAGS_trajectory_time_length;  // 8.0
  const int num_of_knots = static_cast<int>(total_time / delta_t) + 1;

  PiecewiseJerkSpeedOptimizer piecewise_jerk_speed_optimizer(num_of_knots,
                                                             delta_t, init_s);
  // weight
  const auto& weight_config =
      config_.speed_optimizer_config().default_weights();
  piecewise_jerk_speed_optimizer.set_weight(weight_config);

  // s bounds
  std::vector<std::pair<double, double>> s_bounds;
  double max_s = init_s[1] * FLAGS_trajectory_time_length +
                 0.5 * FLAGS_longitudinal_acceleration_upper_bound *
                     FLAGS_trajectory_time_length *
                     FLAGS_trajectory_time_length;
  for (int i = 0; i < num_of_knots; i++) {
    const double t = i * delta_t;
    double s_lower_bound = 0.0;
    double s_upper_bound = max_s;

    PathDecision* path_decision = reference_line_info_->path_decision();
    for (const auto* obstacle_ptr : path_decision->obstacles().Items()) {
      const auto& st_boundary = obstacle_ptr->st_boundary();
      const auto& lon_decision = obstacle_ptr->longitudinal_decision();

      double s_lower = 0.0;
      double s_upper = 0.0;
      if (!st_boundary.GetUnblockSRange(t, s_lower, s_upper, lon_decision)) {
        continue;
      }

      if (lon_decision.has_stop()) {
        s_upper_bound = std::fmin(s_upper_bound, s_upper - kStopBuffer);
      } else if (lon_decision.has_follow()) {
        s_upper_bound = std::fmin(s_upper_bound, s_upper - kFollowBuffer);
      } else if (lon_decision.has_overtake()) {
        s_lower_bound = std::fmax(s_lower_bound + kOverTakeBuffer, s_lower);
      } else if (lon_decision.has_yield()) {
        s_upper_bound = std::fmin(s_upper_bound, s_upper - kYieldBuffer);
      }

      if (s_lower_bound > s_upper_bound) {
        AERROR << "s_lower_bound larger than s_upper_bound on STGraph";
        // speed_data->clear();
        // return false;
        s_upper_bound = s_lower_bound;
      }
    }
    s_bounds.emplace_back(s_lower_bound, s_upper_bound);
  }
  s_bounds_.clear();
  piecewise_jerk_speed_optimizer.set_s_bounds(s_bounds);
  s_bounds_.assign(s_bounds.begin(), s_bounds.end());
  // dds bounds
  piecewise_jerk_speed_optimizer.set_dds_bounds(
      FLAGS_comfortable_longitudinal_acceleration_lower_bound,
      FLAGS_comfortable_longitudinal_acceleration_upper_bound);

  // ddds_bounds //jerk
  piecewise_jerk_speed_optimizer.set_ddds_bounds(
      FLAGS_comfortable_longitudinal_jerk_lower_bound,
      FLAGS_comfortable_longitudinal_jerk_upper_bound);

  // slack_bounds
  const double dds_slack_lower_bound =
      FLAGS_longitudinal_acceleration_lower_bound -
      FLAGS_comfortable_longitudinal_acceleration_lower_bound;
  const double dds_slack_upper_bound =
      FLAGS_longitudinal_acceleration_upper_bound -
      FLAGS_comfortable_longitudinal_acceleration_upper_bound;
  const double ddds_slack_lower_bound =
      FLAGS_longitudinal_jerk_lower_bound -
      FLAGS_comfortable_longitudinal_jerk_lower_bound;
  const double ddds_slack_upper_bound =
      FLAGS_longitudinal_jerk_upper_bound -
      FLAGS_comfortable_longitudinal_jerk_upper_bound;
  piecewise_jerk_speed_optimizer.set_dds_slack_bounds(dds_slack_lower_bound,
                                                      dds_slack_upper_bound);
  piecewise_jerk_speed_optimizer.set_ddds_slack_bounds(ddds_slack_lower_bound,
                                                       ddds_slack_upper_bound);

  // s_ref   // ds_ref  // kappa_ref  // warm_start_val
  std::vector<double> s_ref(num_of_knots);
  std::vector<double> ds_ref(num_of_knots);
  std::vector<double> kappa_ref(num_of_knots);
  std::vector<double> warm_start_val(5 * num_of_knots - 1, 0.0);

  for (int i = 0; i < num_of_knots; i++) {
    const double t = i * delta_t;

    pnc::SpeedPoint speed_point;
    if (!reference_speed_data.EvaluateByTime(t, &speed_point)) {
      AERROR << "EvaluateByTime error, t = " << t;
      return false;
    }

    ds_ref[i] = speed_point.v();
    s_ref[i] = speed_point.s();
    const pnc::PathPoint& path_point =
        path_data.GetPathPointByS(speed_point.s());
    kappa_ref[i] = path_point.kappa();

    warm_start_val[i] = speed_point.s();
    warm_start_val[num_of_knots + i] = speed_point.v();
    warm_start_val[2 * num_of_knots + i] = speed_point.a();
  }
  piecewise_jerk_speed_optimizer.set_s_ref(s_ref);
  piecewise_jerk_speed_optimizer.set_ds_ref(ds_ref);
  piecewise_jerk_speed_optimizer.set_kappa_ref(kappa_ref);
  piecewise_jerk_speed_optimizer.set_warm_start_val(warm_start_val);

  // ds_bounds
  // static std::vector<std::pair<double, double>> ds_bounds;
  ds_bounds_.clear();
  for (int i = 0; i < num_of_knots; i++) {
    const double speed_limit = st_graph_data.speed_limit().GetSpeedLimitByS(s_ref[i]);
    const double min_speed = MinVelocity(i * 0.1, init_s[1], init_s[2]);
    const double upper_bound =
        std::fmax(std::fmax(speed_limit, min_speed), 0.0);
    ds_bounds_.emplace_back(0.0, upper_bound);
  }
  piecewise_jerk_speed_optimizer.set_ds_bounds(ds_bounds_);

  // AERROR << st_graph_data.speed_limit().DebugString();

  // optimize and get speed data
  std::vector<double> opt_s;
  std::vector<double> opt_ds;
  std::vector<double> opt_dds;
  if (!piecewise_jerk_speed_optimizer.Optimize(&opt_s, &opt_ds, &opt_dds)) {
    AERROR << "Optimize of PiecewiseJerkSpeedOptimizer ERROR";
    speed_data->clear();
    return false;
  }

  speed_data->clear();
  speed_data->AppendSpeedPoint(0.0, opt_s[0], opt_ds[0], opt_dds[0], 0.0);
  for (int i = 1; i < num_of_knots; i++) {
    if (opt_ds[i] <= 0.0) {
      for (int j = i; j < num_of_knots; j++) {
        speed_data->AppendSpeedPoint(delta_t * j, opt_s[i - 1], 0.0, 0.0, 0.0);
      }
      break;
    }
    speed_data->AppendSpeedPoint(delta_t * i, opt_s[i], opt_ds[i], opt_dds[i],
                                 (opt_dds[i] - opt_dds[i - 1]) / delta_t);
  }

  // ADEBUG << speed_data->DebugString();
  reference_line_info->AddCost(
      std::fabs(piecewise_jerk_speed_optimizer.ObjectiveValue()));
#if 0
  if (piecewise_jerk_speed_optimizer.IterNum() > 1500) {
    std::fstream speed_opt_input;
    CreateLogFile(speed_opt_input, "ref_speed_data");
    speed_opt_input << "num_of_knots,init_s,init_v,init_a  iternum = "
                    << piecewise_jerk_speed_optimizer.IterNum() << std::endl;
    speed_opt_input << num_of_knots << "," << init_s[0] << "," << init_s[1]
                    << "," << init_s[2] << std::endl;
    speed_opt_input << "ref_s,ref_v,ref_a,ref_k,s_lb,s_ub,v_lb,v_ub"
                    << std::endl;
    for (int i = 0; i < num_of_knots; i++) {
      speed_opt_input << s_ref[i] << "," << ds_ref[i] << ","
                      << warm_start_val[2 * num_of_knots + i] << ","
                      << kappa_ref[i] << "," << s_bounds[i].first << ","
                      << s_bounds[i].second << "," << ds_bounds_[i].first << ","
                      << ds_bounds_[i].second << std::endl;
    }
    speed_opt_input.close();
  }
#endif

  DrawDebugInfo();
  return true;
}

double SpeedOptimizer::MinVelocity(double t, double init_v, double init_a) {
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

std::string SpeedOptimizer::DebugString() const {
  std::stringstream ss;
  ss << "SpeedOptimizer Task DebugString:\n";
  ss << "a bound: [" << FLAGS_longitudinal_acceleration_lower_bound << " , "
     << FLAGS_longitudinal_acceleration_upper_bound << "]";
  ss << "  a comfortable bound: ["
     << FLAGS_comfortable_longitudinal_acceleration_lower_bound << " , "
     << FLAGS_comfortable_longitudinal_acceleration_upper_bound << "]\n";
  ss << "jerk bound: [" << FLAGS_longitudinal_jerk_lower_bound << " , "
     << FLAGS_longitudinal_jerk_upper_bound << "]";
  ss << "  jerk comfortable bound: ["
     << FLAGS_comfortable_longitudinal_jerk_lower_bound << " , "
     << FLAGS_comfortable_longitudinal_jerk_upper_bound << "]\n";
  ss << "a slack bound: ["
     << FLAGS_longitudinal_acceleration_lower_bound -
            FLAGS_comfortable_longitudinal_acceleration_lower_bound
     << " , "
     << FLAGS_longitudinal_acceleration_upper_bound -
            FLAGS_comfortable_longitudinal_acceleration_upper_bound
     << "]\n";
  ss << "jerk slack bound: ["
     << FLAGS_longitudinal_jerk_lower_bound -
            FLAGS_comfortable_longitudinal_jerk_lower_bound
     << " , "
     << FLAGS_longitudinal_jerk_upper_bound -
            FLAGS_comfortable_longitudinal_jerk_upper_bound
     << "]\n";

  const auto& weight_config =
      config_.speed_optimizer_config().default_weights();
  std::string out_str;
  ::google::protobuf::TextFormat::PrintToString(weight_config, &out_str);
  ss << out_str;

  return ss.str();
}

void SpeedOptimizer::DrawDebugInfo() {
  pnc::Debug* debug_info = internal_->mutable_debug_info();
  int size = debug_info->charts_size();
  for (int i = 0; i < size; i++) {
    auto iter = debug_info->mutable_charts(i);
    if (iter->title() == "s-t") {
      auto line_ptr = iter->add_line();
      line_ptr->set_label("QP");
      (*line_ptr->mutable_properties())["color"] = "r";
      // (*line_ptr->mutable_properties())["linestyle"] = "dashed";
      for (const auto& speed_point : reference_line_info_->speed_data()) {
        pnc::Point2D* point2d = line_ptr->add_point();
        point2d->set_x(speed_point.t());
        point2d->set_y(speed_point.s());
      }

      if (reference_line_info_->st_graph_data().st_boundaries().size() > 0) {
        // s_up
        auto s_upper_line_ptr = iter->add_line();
        s_upper_line_ptr->set_label("s_ub");
        (*s_upper_line_ptr->mutable_properties())["color"] = "k";
        (*s_upper_line_ptr->mutable_properties())["linestyle"] = "dashed";
        for (int j = 0; j < s_bounds_.size(); j++) {
          pnc::Point2D* point2d = s_upper_line_ptr->add_point();
          point2d->set_x(j * 0.1);
          point2d->set_y(s_bounds_[j].second);
        }
        // s_lb
        auto s_lower_line_ptr = iter->add_line();
        s_lower_line_ptr->set_label("s_lb");
        (*s_lower_line_ptr->mutable_properties())["color"] = "k";
        (*s_lower_line_ptr->mutable_properties())["linestyle"] = "dashed";
        for (int j = 1; j < s_bounds_.size() - 1; j++) {
          pnc::Point2D* point2d = s_lower_line_ptr->add_point();
          point2d->set_x(j * 0.1);
          point2d->set_y(s_bounds_[j].first);
        }
      }
    } else if (iter->title() == "v-t") {
      auto line_ptr = iter->add_line();
      line_ptr->set_label("QP");
      (*line_ptr->mutable_properties())["color"] = "r";
      // (*line_ptr->mutable_properties())["linestyle"] = "dashed";
      for (const auto& speed_point : reference_line_info_->speed_data()) {
        pnc::Point2D* point2d = line_ptr->add_point();
        point2d->set_x(speed_point.t());
        point2d->set_y(speed_point.v());
      }
      // speed bound
      auto speed_line_ptr = iter->add_line();
      speed_line_ptr->set_label("speed_bound");
      (*speed_line_ptr->mutable_properties())["color"] = "g";
      (*speed_line_ptr->mutable_properties())["linestyle"] = "dashed";
      for (int j = 0; j < ds_bounds_.size(); j++) {
        pnc::Point2D* point2d = speed_line_ptr->add_point();
        point2d->set_x(j * 0.1);
        point2d->set_y(ds_bounds_[j].second);
      }

    } else if (iter->title() == "a-t") {
      auto line_ptr = iter->add_line();
      line_ptr->set_label("QP");
      (*line_ptr->mutable_properties())["color"] = "r";
      // (*line_ptr->mutable_properties())["linestyle"] = "dashed";
      for (const auto& speed_point : reference_line_info_->speed_data()) {
        pnc::Point2D* point2d = line_ptr->add_point();
        point2d->set_x(speed_point.t());
        point2d->set_y(speed_point.a());
      }

      // // acc upper bound
      // auto acc_upper_line_ptr = iter->add_line();
      // acc_upper_line_ptr->set_label("acc_ub");
      // (*acc_upper_line_ptr->mutable_properties())["color"] = "g";
      // (*acc_upper_line_ptr->mutable_properties())["linestyle"] = "dashed";
      // for (int j = 0; j < 81; j++) {
      //   pnc::Point2D* point2d = acc_upper_line_ptr->add_point();
      //   point2d->set_x(j * 0.1);
      //   point2d->set_y(FLAGS_longitudinal_acceleration_upper_bound);
      // }

      // // acc lower bound
      // auto acc_lower_line_ptr = iter->add_line();
      // acc_lower_line_ptr->set_label("acc_lb");
      // (*acc_lower_line_ptr->mutable_properties())["color"] = "b";
      // (*acc_lower_line_ptr->mutable_properties())["linestyle"] = "dashed";
      // for (int j = 0; j < 81; j++) {
      //   pnc::Point2D* point2d = acc_lower_line_ptr->add_point();
      //   point2d->set_x(j * 0.1);
      //   point2d->set_y(FLAGS_longitudinal_acceleration_lower_bound);
      // }
    }
  }

  // draw jerk
  // pnc::Chart* jerk_chart = debug_info->add_charts();
  // jerk_chart->set_title("jerk-t");
  // jerk_chart->mutable_options()->mutable_x()->set_min(0);
  // jerk_chart->mutable_options()->mutable_x()->set_max(9);
  // jerk_chart->mutable_options()->mutable_x()->set_label_string("t(s)");
  // jerk_chart->mutable_options()->mutable_y()->set_min(
  //     FLAGS_longitudinal_jerk_lower_bound * 1.1);
  // jerk_chart->mutable_options()->mutable_y()->set_max(
  //     FLAGS_longitudinal_jerk_upper_bound * 1.1);
  // jerk_chart->mutable_options()->mutable_y()->set_label_string("jerk(m/s^3)");
  // auto qp_jerk_line = jerk_chart->add_line();
  // qp_jerk_line->set_label("QP");
  // (*qp_jerk_line->mutable_properties())["color"] = "r";
  // for (const auto& speed_point : reference_line_info_->speed_data()) {
  //   pnc::Point2D* point2d = qp_jerk_line->add_point();
  //   point2d->set_x(speed_point.t());
  //   point2d->set_y(speed_point.da());
  // }
}

void SpeedOptimizer::CreateLogFile(std::fstream& log_file, std::string name) {
  time_t raw_time;
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);

  std::string dir = getenv("HOME");
  dir += "/speed_optimizer_log";

  if (access(dir.c_str(), 0) == -1) {
    std::string command = "mkdir -p " + dir;
    system(command.c_str());
  }

  // char time_str[80];
  // strftime(time_str, 80, "%F-%H%M%S_", &time_tm);
  // dir = dir + "/" + time_str + name + ".csv";
  dir = dir + "/" + name + ".csv";

  log_file.close();
  log_file.open(dir.c_str(), std::ios_base::out);
  log_file.flags(std::ios::fixed);
}

}  // namespace planning
}  // namespace xju
