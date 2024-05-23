#!/bin/bash

source /home/ws/install/setup.bash

control_mode="control_mode:=p" # p(perfect_control), k(kinematics_control) or t(torque_control)

use_sim_time="use_sim_time:=true" # true or false

publish_clock="publish_clock:=true" # true or false

publish_em_obs="publish_em_obs:=false" # true or false

#ros2 bag record -a -x /clock -o /home/ws/bag/`date +"%Y%m%d-%H-%M-%S"` &

rviz2 -d /home/ws/src/pnc/simulator/rviz_xju_panel/rviz_config/config.rviz &

sleep 1

ros2 launch dynamic_model_simulator_package dynamic_model_simulator.launch.xml $control_mode $use_sim_time &

ros2 launch perception_simulator_package perception_simulator.launch.xml $use_sim_time $publish_em_obs &

ros2 launch emlanes_simulator_package emlanes_simulator.launch.xml $use_sim_time &

ros2 launch planning_package planning.launch.xml $use_sim_time &

ros2 launch debug_tool_package debug_tool.launch.xml $control_mode $publish_clock

pkill -f dynamic_mode
pkill -f perception_simu
pkill -f rviz2
pkill -f emlanes_simulat
pkill -f planning
pkill -f planning_debug
pkill -f control_debug
pkill -f control_node
pkill -f prediction
#pkill -f rosbag
