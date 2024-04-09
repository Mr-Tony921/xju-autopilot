/******************************************************************************
 * Copyright 2023 The XJU AutoPilot Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include "gflags/gflags.h"

namespace xju {
namespace planning {
DECLARE_double(planning_loop_rate);

// scenario config file
DECLARE_string(scenario_lane_follow_config_file);
DECLARE_string(scenario_emergency_pull_over_config_file);
DECLARE_string(scenario_emergency_stop_config_file);
DECLARE_string(scenario_road_change_config_file);

// scenario condition
DECLARE_double(road_change_in_distance_threshold);
DECLARE_double(road_change_out_distance_threshold);

// lane decider default config file
DECLARE_string(lane_decider_default_config_file);

// replan
DECLARE_bool(enable_trajectory_stitcher);
DECLARE_double(replan_longitudinal_distance_threshold);
DECLARE_double(replan_lateral_distance_threshold);
DECLARE_uint64(trajectory_stitching_preserved_length);
DECLARE_double(speed_lon_decision_horizon);

// parameters for trajectory planning
DECLARE_double(default_cruise_speed);
DECLARE_double(road_change_cruise_speed);
DECLARE_double(trajectory_time_length);
DECLARE_double(trajectory_time_min_interval);
DECLARE_double(trajectory_time_max_interval);
DECLARE_double(trajectory_time_high_density_period);

// parameters for trajectory sanity check
DECLARE_double(speed_lower_bound);
DECLARE_double(speed_upper_bound);

DECLARE_double(longitudinal_acceleration_lower_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);
DECLARE_double(comfortable_longitudinal_acceleration_lower_bound);
DECLARE_double(comfortable_longitudinal_acceleration_upper_bound);
DECLARE_double(longitudinal_acceleration_upper_bound);

DECLARE_double(lateral_acceleration_bound);

DECLARE_double(longitudinal_jerk_lower_bound);
DECLARE_double(longitudinal_jerk_upper_bound);
DECLARE_double(comfortable_longitudinal_jerk_lower_bound);
DECLARE_double(comfortable_longitudinal_jerk_upper_bound);
DECLARE_double(lateral_jerk_bound);

DECLARE_double(kappa_bound);

// parameters for path optimization
DECLARE_bool(enable_gradually_discretization);
DECLARE_double(gradually_discretization_splice);
DECLARE_double(heading_error_bound);

DECLARE_double(virtual_stop_wall_length);
DECLARE_double(static_obstacle_speed_threshold);

DECLARE_double(obstacle_lat_buffer);

DECLARE_double(obstacle_lon_start_buffer);

DECLARE_double(obstacle_lon_end_buffer);
DECLARE_double(max_centripetal_acceleration);

DECLARE_double(keep_lane_id_range);

// delay of subscribe topic
DECLARE_bool(ignore_localize_message_delay);
DECLARE_double(localize_msg_delay_threshold);
DECLARE_bool(ignore_chassis_message_delay);
DECLARE_double(chassis_msg_delay_threshold);
DECLARE_bool(ignore_emlanes_message_delay);
DECLARE_double(emlanes_msg_delay_threshold);
DECLARE_bool(ignore_prediction_message_delay);
DECLARE_double(prediction_msg_delay_threshold);
DECLARE_bool(ignore_routing_message_delay);
DECLARE_double(routing_msg_delay_threshold);

DECLARE_double(fallback_counter_threshold);
} // namespace planning
} // namespace xju
