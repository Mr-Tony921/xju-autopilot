/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#include "planning/common/planning_gflags/planning_gflags.h"

#include <cmath>

namespace xju {
namespace planning {

DEFINE_double(planning_loop_rate, 10, "planning module loop rate, hz");

// scenario config file
DEFINE_string(scenario_lane_follow_config_file, 
    "configs/planning/scenarios/lane_follow_scenario_config.pb.txt", 
    "lane follow scenario config file.");
DEFINE_string(scenario_road_change_config_file, 
    "configs/planning/scenarios/road_change_scenario_config.pb.txt", 
    "road change scenario config file.");

// scenario condition
DEFINE_double(road_change_in_distance_threshold, 2000.0, 
    "Distance when change into RoadChange scenario.");
DEFINE_double(road_change_out_distance_threshold, 2500.0, 
    "Distance when out from RoadChange scenario.");
DEFINE_string(scenario_emergency_pull_over_config_file,
              "configs/planning/scenarios/emergency_pull_over_config.pb.txt",
              "emergency pull over scenario config file.");
DEFINE_string(scenario_emergency_stop_config_file,
              "configs/planning/scenarios/emergency_stop_config.pb.txt",
              "emergency stop scenario config file.");

// lane decider default config file
DEFINE_string(lane_decider_default_config_file,
    "configs/planning/lane_decider_default_config_file.pb.txt", 
    "lane follow scenario config file.");

DEFINE_bool(enable_trajectory_stitcher, true, "enable stitching trajectory.");
DEFINE_double(replan_longitudinal_distance_threshold, 2.0, 
    "The longitudinal distance threshold of replan.");
DEFINE_double(replan_lateral_distance_threshold, 0.5, 
    "The lateral distance threshold of replan.");
DEFINE_uint64(trajectory_stitching_preserved_length, 20,
              "preserved points number in trajectory stitching");


DEFINE_double(speed_lon_decision_horizon, 200.0,
              "Longitudinal horizon for speed decision making (meter)");

DEFINE_double(default_cruise_speed, 5.0, "default cruise speed");
DEFINE_double(road_change_cruise_speed, 11.1, "road change cruise speed");
DEFINE_double(trajectory_time_length, 8.0, "Trajectory time length");
DEFINE_double(
    trajectory_time_min_interval, 0.02,
    "(seconds) Trajectory time interval when publish. The is the min value.");
DEFINE_double(
    trajectory_time_max_interval, 0.1,
    "(seconds) Trajectory time interval when publish. The is the max value.");
DEFINE_double(
    trajectory_time_high_density_period, 1.0,
    "(seconds) Keep high density in the next this amount of seconds. ");

DEFINE_double(speed_lower_bound, -0.1, "The lowest speed allowed.");
DEFINE_double(speed_upper_bound, 40.0, "The highest speed allowed.");

DEFINE_double(longitudinal_acceleration_lower_bound, -6.0,
              "The lowest longitudinal acceleration allowed.");
DEFINE_double(longitudinal_acceleration_upper_bound, 4.0,
              "The highest longitudinal acceleration allowed.");
DEFINE_double(comfortable_longitudinal_acceleration_lower_bound, -3.0,
              "The comfortable longitudinal acceleration.");
DEFINE_double(comfortable_longitudinal_acceleration_upper_bound, 2.0,
              "The comfortable longitudinal acceleration.");
DEFINE_double(lateral_acceleration_bound, 4.0,
              "Bound of lateral acceleration; symmetric for left and right");

DEFINE_double(longitudinal_jerk_lower_bound, -4.0,
              "The lower bound of longitudinal jerk.");
DEFINE_double(longitudinal_jerk_upper_bound, 2.0,
              "The upper bound of longitudinal jerk.");
DEFINE_double(comfortable_longitudinal_jerk_lower_bound, -2.0,
              "The comfortable bound of longitudinal jerk.");
DEFINE_double(comfortable_longitudinal_jerk_upper_bound, 1.0,
              "The comfortable bound of longitudinal jerk.");
DEFINE_double(lateral_jerk_bound, 4.0,
              "Bound of lateral jerk; symmetric for left and right");

DEFINE_double(kappa_bound, 0.1979, "The bound for trajectory curvature");

// parameters for path optimization
DEFINE_bool(enable_gradually_discretization, false, 
            "enable gradually discretization in path optimization");
DEFINE_double(gradually_discretization_splice, 50.0, 
            "gradually discretization splice distance");
DEFINE_double(heading_error_bound, M_PI_2, 
            "heading error between car and reference line");

DEFINE_double(virtual_stop_wall_length, 0.5,"virtual_stop_wall_length");

DEFINE_double(max_centripetal_acceleration,20.0,"max_centripetal_acceleration");

DEFINE_double(static_obstacle_speed_threshold, 0.5,
              "The speed threshold to decide whether an obstacle is static "
              "or not.");
              
DEFINE_double(obstacle_lat_buffer, 0.4,
              "obstacle lateral buffer (meters) for deciding path boundaries");

DEFINE_double(obstacle_lon_start_buffer, 3.0,
              "obstacle longitudinal start buffer (meters) for deciding "
              "path boundaries");

DEFINE_double(obstacle_lon_end_buffer, 2.0,
              "obstacle longitudinal end buffer (meters) for deciding "
              "path boundaries");
              
DEFINE_double(keep_lane_id_range,2.7, 
              "range in ego lane between ego positon to farmost edge");
// delay of subscribe topic
DEFINE_bool(ignore_localize_message_delay, false, "ignore localize message delay.");
DEFINE_double(localize_msg_delay_threshold, 0.2, "location msg delay threshold, unit: s.");
DEFINE_bool(ignore_chassis_message_delay, false, "ignore chassis message delay.");
DEFINE_double(chassis_msg_delay_threshold, 0.2, "chassis msg delay threshold, unit: s.");
DEFINE_bool(ignore_emlanes_message_delay, false, "ignore emlanes message delay.");
DEFINE_double(emlanes_msg_delay_threshold, 0.5, "emlanes msg delay threshold, unit: s.");
DEFINE_bool(ignore_prediction_message_delay, false, "ignore prediction message delay.");
DEFINE_double(prediction_msg_delay_threshold, 0.3, "prediction msg delay threshold, unit: s.");
DEFINE_bool(ignore_routing_message_delay, true, "ignore routing message delay.");
DEFINE_double(routing_msg_delay_threshold, 1.0, "routing msg delay threshold, unit: s.");

DEFINE_double(fallback_counter_threshold, 1, "fallback counter threshold");
}  // namespace planning
} // namespace xju
