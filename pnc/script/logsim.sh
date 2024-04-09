#!/bin/bash

source /home/ws/install/setup.bash

prediction_remap="/perception/em/obstacles:=/origin/perception/em/obstacles
                  /prediction/viz/object:=/prediction/viz/object"

planning_remap="/pnc/planning:=/origin/pnc/planning
                /pnc/debug_marker:=/origin/pnc/debug_marker"

control_remap="/pnc/control:=/origin/pnc/control
               control/predicted_trajectory_topic:=origin/control/predicted_trajectory_topic"

debug_tool_remap="/debug_tool/viz/ego_model:=/origin/debug_tool/viz/ego_model 
                  /debug_tool/planning_overlay:=/origin/debug_tool/planning_overlay
                  /debug_tool/chassis_overlay:=/origin/debug_tool/chassis_overlay"
                  
remap="/test_topic:=/origin/test_topic"
        
bag_dir=""

prediction_cmd=false
control_cmd=false
planning_cmd=false
debug_tool_cmd=false

play_speed=1.0

while [ $# -gt 0 ]; do
  case $1 in
    --bag)
      bag_dir=$2
      echo "bag dir is ${2}"
      shift 2;
      ;;
    -b)
      bag_dir=$2
      echo "bag dir is ${2}"
      shift 2;
      ;;
    -pre)
      prediction_cmd=true
      remap="$remap $prediction_remap"
      shift 1;
      ;;
    -p)
      planning_cmd=true
      debug_tool_cmd=true
      remap="$remap $planning_remap $debug_tool_remap"
      shift 1;
      ;;
    -c)
      control_cmd=true
      debug_tool_cmd=true
      remap="$remap $control_remap $debug_tool_remap"
      shift 1;
      ;;
    -pnc)
      planning_cmd=true
      control_cmd=true
      debug_tool_cmd=true
      remap="$remap $planning_remap $control_remap $debug_tool_remap"
      shift 1;
      ;;
    -ppnc)
      prediction_cmd=true
      planning_cmd=true
      control_cmd=true
      debug_tool_cmd=true
      remap="$remap $planning_remap $control_remap $prediction_remap $debug_tool_remap"
      shift 1;
      ;;        
    -r)
      play_speed=$2
      echo "play speed is ${2}"
      shift 2;
      ;;
    --rate)
      play_speed=$2
      echo "play speed is ${2}"
      shift 2;
      ;;
    *)
      echo "unknown option: $1"
      exit
      ;;
  esac
done

if [[ $bag_dir == "" ]]
  then
    echo "bag_dir null !"
    exit 
fi

if [ ! -d $bag_dir ]
  then
    echo "$bag_dir no such directory !"
    exit 
fi

bag_topics=""
while read line
do      
  if [[ ${line:0:5} == "name:" ]]
    then
      if [[ ${line:0:12} != "name: /clock" ]]
        then
          bag_topics="$bag_topics ${line#*:}"
      fi
  fi
done < $bag_dir/metadata.yaml

rviz2 -d $(ros2 pkg prefix rviz_xju_panel)/share/rviz_xju_panel/rviz_config/config.rviz &

if [ $prediction_cmd == true ]
  then
    ros2 launch prediction_package prediction.launch.xml use_sim_time:=true &
fi

if [ $control_cmd == true ]
  then
    ros2 launch control_package control.launch.xml use_sim_time:=true &
fi

if [ $planning_cmd == true ]
  then
    ros2 launch planning_package planning.launch.xml use_sim_time:=true &
fi

if [ $debug_tool_cmd == true ]
  then
    ros2 launch debug_tool_package debug_tool.launch.xml control_mode:=k publish_clock:=false &
fi

ros2 bag play $bag_dir --topic $bag_topics --clock --remap $remap -r $play_speed

pkill -f rviz2
pkill -f planning_debug
pkill -f control_debug
pkill -f planning
pkill -f control
pkill -f prediction