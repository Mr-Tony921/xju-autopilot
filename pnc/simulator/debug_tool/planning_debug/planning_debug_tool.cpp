
/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/
#include "simulator/debug_tool/planning_debug/planning_debug_tool.h"

namespace xju {
namespace simulator {

void DebugTool::InitParam() {
  // load yaml param
  this->declare_parameter<std::string>("gflag_config_file_path", "");
  this->declare_parameter<std::string>("control_mode", "KINEMATICS");
  this->declare_parameter<std::string>("vehicle_config_file", "");
  this->declare_parameter<bool>("publish_clock", false);
  std::string gflag_config_file_path, vehicle_config_file;
  this->get_parameter("gflag_config_file_path", gflag_config_file_path);
  this->get_parameter("control_mode", control_mode_);
  if (control_mode_ == "p") {
    control_mode_ = "PERFECT";
  } else if (control_mode_ == "k") {
    control_mode_ = "KINEMATICS";
  } else if (control_mode_ == "t") {
    control_mode_ = "TORQUE";
  }
  this->get_parameter("vehicle_config_file", vehicle_config_file);
  this->get_parameter("publish_clock", publish_clock_);

  // init gflag param
  xju::pnc::File::GetGflagConfig(gflag_config_file_path);
  xju::pnc::FLAGS_vehicle_config_file = vehicle_config_file;
  // get vehicle config
  auto vehicle_config = xju::pnc::VehicleConfigProvider::GetConfig();

  rear_to_back_ = vehicle_config.back_overhang_length();
  car_length_ = vehicle_config.length();
  car_width_ = vehicle_config.width();
  car_height_ = vehicle_config.height();

  rear_axle_to_hitch_ =
      vehicle_config.rear_axle_to_hitch();  // d>0 lizhou //d<0 guozhou
}

void DebugTool::SimCmdCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (msg->header.frame_id == "planning_chart") {
    ADEBUG << "Receive SimCmd msg: " << msg->header.frame_id << "  "
           << msg->pose.covariance[0];
    if (msg->pose.covariance[0] > 0) {
      open_debug = true;
    } else {
      open_debug = false;
    }
  } else if (msg->header.frame_id == "change_sim_rate") {
    ADEBUG << "Receive SimCmd msg: " << msg->header.frame_id
           << "  sim_rate:" << msg->pose.covariance[0];
    sim_rate_ = msg->pose.covariance[0];
  }
}

void DebugTool::LocalizationCallback(
    const localization_msgs::msg::LocalizeOutput::SharedPtr msg) {
  if (!localization_) {
    localization_ =
        std::make_unique<localization_msgs::msg::LocalizeOutput>(*msg);
  } else {
    *localization_ = *msg;
  }
}

void DebugTool::PlanningTrajCallback(
    const planning_msgs::msg::Planning::SharedPtr msg) {
  if (!planning_traj_) {
    planning_traj_ = std::make_unique<planning_msgs::msg::Planning>(*msg);
  } else {
    *planning_traj_ = *msg;
  }
}

void DebugTool::GenerateChart() {
  std::chrono::seconds sleep_duration(1);
  while (rclcpp::ok()) {
    if (!open_debug) {
      if (figure_has_opened) {
        plt::close();
        figure_has_opened = false;
      }
      std::this_thread::sleep_for(sleep_duration);
      continue;
    }

    if (!planning_traj_) {
      std::this_thread::sleep_for(sleep_duration);
      continue;
    }

    const auto planning_traj = *planning_traj_;
    const auto& charts = planning_traj.debug.charts;
    int size = charts.size();
    if (size == 0) {
      continue;
    } else {
      if (size <= 4) {
        row_num = size;
        col_num = 1;
      } else if (size == 5 || size == 6) {
        row_num = 3;
        col_num = 2;
      } else if (size == 7 || size == 8) {
        row_num = 4;
        col_num = 2;
      } else if (size == 9) {
        row_num = 3;
        col_num = 3;
      } else {
        row_num = 4;
        if ((size % 4) != 0) {
          col_num = int(size / 4) + 1;
        } else {
          col_num = int(size / 4);
        }
      }
    }

    if (!figure_has_opened) {
      plt::figure_size("Planning Debug Charts", 400 * col_num, 400 * row_num);
      plt::ioff();
      figure_has_opened = true;
    }
    plt::clf();
    plt::figtext(
        0, 0, pnc::Time::NowInDateWithSec(planning_traj.header.timestamp_sec));
    for (int i = 0; i < size; i++) {
      plt::subplot(row_num, col_num, i + 1);
      // plt::subplots_adjust({{"hspace", 0.4}});
      plt::title(charts[i].title);
      plt::xlim(charts[i].options.x.min, charts[i].options.x.max);
      plt::ylim(charts[i].options.y.min, charts[i].options.y.max);
      if (charts[i].title == "x-y") {
        plt::axis("equal");
      }
      // plt::axis("square");
      plt::xlabel(charts[i].options.x.label_string);
      plt::ylabel(charts[i].options.y.label_string);
      // lines
      for (const auto& line : charts[i].lines) {
        std::vector<double> xx, yy;
        for (const auto& point : line.point) {
          xx.push_back(point.x);
          yy.push_back(point.y);
        }
        auto color = ToColorKeyWord(line.color.r, line.color.g, line.color.b);
        auto line_type = ToLineTypeKeyWord(line.line_type);
        plt::plot(xx, yy,
                  {{"color", color},
                   {"linestyle", line_type},
                   {"label", line.label}});
        if (!line.hide_label_in_legend) {
          plt::legend();
        }
      }
      // polygons
      for (const auto& polygon : charts[i].polygons) {
        std::vector<double> xx, yy;
        for (const auto& point : polygon.point) {
          xx.push_back(point.x);
          yy.push_back(point.y);
        }
        if (!xx.empty()) {
          xx.push_back(xx.front());
          yy.push_back(yy.front());
        }
        auto color =
            ToColorKeyWord(polygon.color.r, polygon.color.g, polygon.color.b);
        plt::plot(xx, yy, {{"color", color}, {"label", polygon.label}});
        if (!polygon.hide_label_in_legend) {
          plt::legend();
        }
      }
    }

    plt::tight_layout();
    // plt::draw();
    plt::pause(0.000011);
  }
}

void DebugTool::PublishClock() {
  while (rclcpp::ok()) {
    auto t = this->get_clock()->now();
    double sec = double(msg_.clock.sec) + double(msg_.clock.nanosec) / 1.0e9 +
                 (t - real_time_).seconds() * sim_rate_;
    msg_.clock.sec = sec;
    msg_.clock.nanosec = 1.0e9 * (sec - msg_.clock.sec);
    clock_pub_->publish(msg_);
    real_time_ = t;
    usleep(1000);
  }
}

std::string DebugTool::ToColorKeyWord(const float r, const float g,
                                      const float b) {
  if (r < 1e-3 && g < 1e-3 && b < 1e-3) {
    return "black";
  }
  if (r > 1e-3) {
    return "red";
  }
  if (g > 1e-3) {
    return "green";
  }
  if (b > 1e-3) {
    return "blue";
  }
  return "black";
}

std::string DebugTool::ToLineTypeKeyWord(const uint val) {
  if (val == 0) {
    return "solid";
  }
  if (val == 1) {
    return "dashed";
  }
  if (val == 2) {
    return "dotted";
  }
  return "solid";
}

void DebugTool::PublishOverlayText() {
  // planning overlay
  rviz_2d_overlay_msgs::msg::OverlayText planning_ov_msg;
  std::stringstream planning_ss;
  planning_ss << std::fixed;
  if (planning_traj_) {
    planning_ss << "traj_timestamp: "
                << pnc::Time::NowInDateWithSec(
                       planning_traj_->header.timestamp_sec)
                << "\n";
    planning_ss << "scenario_type:  ";
    if (planning_traj_->scenario_type ==
        planning_msgs::msg::Planning::LANE_FOLLOW) {
      planning_ss << "LANE_FOLLOW";
    } else if (planning_traj_->scenario_type ==
               planning_msgs::msg::Planning::ROAD_CHANGE) {
      planning_ss << "ROAD_CHANGE";
    } else if (planning_traj_->scenario_type ==
               planning_msgs::msg::Planning::EMERGENCY_PULL_OVER) {
      planning_ss << "EMERGENCY_PULL_OVER";
    } else if (planning_traj_->scenario_type ==
               planning_msgs::msg::Planning::EMERGENCY_STOP) {
      planning_ss << "EMERGENCY_STOP";
    }
    planning_ss << "\n";
    planning_ss << "stage_type:  ";
    if (planning_traj_->stage_type == planning_msgs::msg::Planning::NO_STAGE) {
      planning_ss << "NO_STAGE";
    } else if (planning_traj_->stage_type ==
               planning_msgs::msg::Planning::LANE_FOLLOW_DEFAULT_STAGE) {
      planning_ss << "LANE_FOLLOW_DEFAULT_STAGE";
    } else if (planning_traj_->stage_type ==
               planning_msgs::msg::Planning::ROAD_CHANGE_DEFAULT_STAGE) {
      planning_ss << "ROAD_CHANGE_DEFAULT_STAGE";
    } else if (planning_traj_->stage_type ==
               planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_LANE_CHANGE) {
      planning_ss << "EMERGENCY_PULL_OVER_LANE_CHANGE";
    } else if (planning_traj_->stage_type ==
               planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_SLOW_DOWN) {
      planning_ss << "EMERGENCY_PULL_OVER_SLOW_DOWN";
    } else if (planning_traj_->stage_type ==
               planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_APPROACH) {
      planning_ss << "EMERGENCY_PULL_OVER_APPROACH";
    } else if (planning_traj_->stage_type ==
               planning_msgs::msg::Planning::EMERGENCY_PULL_OVER_STANDBY) {
      planning_ss << "EMERGENCY_PULL_OVER_STANDBY";
    } else if (planning_traj_->stage_type ==
               planning_msgs::msg::Planning::EMERGENCY_STOP_APPROACH) {
      planning_ss << "EMERGENCY_STOP_APPROACH";
    } else if (planning_traj_->stage_type ==
               planning_msgs::msg::Planning::EMERGENCY_STOP_STANDBY) {
      planning_ss << "EMERGENCY_STOP_STANDBY";
    }
    planning_ss << "\n";
    planning_ss << "trajectory_type: ";
    if (planning_traj_->trajectory_type ==
        planning_msgs::msg::Planning::NORMAL) {
      planning_ss << "NORMAL";
    } else if (planning_traj_->trajectory_type ==
               planning_msgs::msg::Planning::PATH_FALLBACK) {
      planning_ss << "PATH_FALLBACK";
    } else if (planning_traj_->trajectory_type ==
               planning_msgs::msg::Planning::SPEED_FALLBACK) {
      planning_ss << "SPEED_FALLBACK";
    } else if (planning_traj_->trajectory_type ==
               planning_msgs::msg::Planning::TRAJECTORY_FALLBACK) {
      planning_ss << "TRAJECTORY_FALLBACK";
    }
    planning_ss << "\n";
    planning_ss << "target_lane_id: " << planning_traj_->target_lane_id << "\n"
                << "lane_id: " << planning_traj_->lane_id << "\n"
                << "is_replan: "
                << (planning_traj_->is_replan ? "true" : "false") << "  "
                << planning_traj_->replan_reason << "\n"
                << "total_path_length: " << planning_traj_->total_path_length
                << "\n"
                << "total_path_time: " << planning_traj_->total_path_time;
    planning_ss << "\n";
    planning_ss << "lat_shift id: " << planning_traj_->lateral_shift_obstacle.id
                << " dir: "
                << (planning_traj_->lateral_shift_obstacle.direction == 0
                        ? "NONE"
                        : (planning_traj_->lateral_shift_obstacle.direction == 1
                               ? "LEFT"
                               : "RIGHT"))
                << "  dis: " << planning_traj_->lateral_shift_obstacle.distance;
    planning_ss << "\n";
    planning_ss << "lane_change status: ";
    if (planning_traj_->lane_change_status.status ==
        planning_msgs::msg::LaneChangeStatus::LANE_FOLLOW) {
      planning_ss << "LANE_FOLLOW";
    } else if (planning_traj_->lane_change_status.status ==
               planning_msgs::msg::LaneChangeStatus::LANE_CHANGE_PREPARE) {
      planning_ss << "LANE_CHANGE_PREPARE";
    } else if (planning_traj_->lane_change_status.status ==
               planning_msgs::msg::LaneChangeStatus::LANE_CHANGE) {
      planning_ss << "LANE_CHANGE";
    }
    planning_ss << " path_id: " << planning_traj_->lane_change_status.path_id;
    planning_ss << "\n";
    planning_ss << "turn_signal: ";
    if (planning_traj_->turn_signal ==
        common_msgs::msg::VehicleSignal::TURN_NONE) {
      planning_ss << "TURN_NONE";
    } else if (planning_traj_->turn_signal ==
               common_msgs::msg::VehicleSignal::TURN_LEFT) {
      planning_ss << "TURN_LEFT";
    } else if (planning_traj_->turn_signal ==
               common_msgs::msg::VehicleSignal::TURN_RIGHT) {
      planning_ss << "TURN_RIGHT";
    }
    planning_ss << "\n";
    planning_ss << "target_speed: " << planning_traj_->target_speed;
    planning_ss << "\n";
    planning_ss << "error_code: " << planning_traj_->error_code;
    planning_ss << "\n";
  }
  planning_ov_msg.text = planning_ss.str();
  planning_ov_msg.fg_color.a = 1.0;
  planning_ov_msg.fg_color.r = 0.96f;
  planning_ov_msg.fg_color.g = 0.22f;
  planning_ov_msg.fg_color.b = 0.06f;
  planning_ov_msg.width = 200;
  planning_ov_msg.height = 40;
  planning_overlay_pub_->publish(planning_ov_msg);

  // chassis overlay
  rviz_2d_overlay_msgs::msg::OverlayText chassis_ov_msg;
  std::stringstream chassis_ss;
  chassis_ss << std::fixed;
  if (localization_) {
    chassis_ss << "x:  " << localization_->local_localize_result.position.x
               << "\n"
               << "y:  " << localization_->local_localize_result.position.y
               << "\n"
               << "Θ:  "
               << localization_->local_localize_result.euler_ypr.z << "\n"
               << "v:  "
               << localization_->local_localize_result.velocity_in_vehicle.x
               << "\n"
               << "a: "
               << localization_->local_localize_result
                      .linear_acceleration_in_vehicle.x;
  }
  chassis_ov_msg.text = chassis_ss.str();
  chassis_ov_msg.fg_color.a = 1.0;
  chassis_ov_msg.fg_color.r = 0.96f;
  chassis_ov_msg.fg_color.g = 0.22f;
  chassis_ov_msg.fg_color.b = 0.06f;
  chassis_ov_msg.width = 200;
  chassis_ov_msg.height = 40;
  chassis_overlay_pub_->publish(chassis_ov_msg);
}
}  // namespace simulator
}  // namespace xju