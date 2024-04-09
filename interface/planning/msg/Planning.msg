# ScenarioType 
uint8 LANE_FOLLOW = 0
uint8 EMERGENCY_PULL_OVER = 1
uint8 EMERGENCY_STOP = 2
uint8 ROAD_CHANGE = 3

# StageType
uint8 NO_STAGE = 0
uint8 LANE_FOLLOW_DEFAULT_STAGE = 1
uint8 ROAD_CHANGE_DEFAULT_STAGE = 2
uint8 EMERGENCY_PULL_OVER_LANE_CHANGE = 20
uint8 EMERGENCY_PULL_OVER_SLOW_DOWN = 21
uint8 EMERGENCY_PULL_OVER_APPROACH = 22
uint8 EMERGENCY_PULL_OVER_STANDBY = 23
uint8 EMERGENCY_STOP_APPROACH = 30
uint8 EMERGENCY_STOP_STANDBY = 31

# TrajectroyType
uint8 NORMAL = 0
uint8 PATH_FALLBACK = 1
uint8 SPEED_FALLBACK = 2
uint8 TRAJECTORY_FALLBACK = 3

common_msgs/Header header

float64 total_path_length
float64 total_path_time

EStop estop

# is_replan == true mean replan triggered
bool is_replan
string replan_reason

# planning result
common_msgs/TrajectoryPoint[] trajectory_point

uint8 trajectory_type

# lane id along current reference line
int32 lane_id

# lane id target reference line
int32 target_lane_id

# scenario info
uint8 scenario_type
uint8 stage_type

# obstacle decision info to visualization
ObstacleDecision[] obstalce_decision

# obstacle decision info to visualization
LateralShiftObstacle lateral_shift_obstacle 

# lane change status
LaneChangeStatus lane_change_status

# vechicle signal info
# assignment using common_msgs/msg/vechicle_signal
uint8 turn_signal

uint16 error_code

float64 target_speed

chart_msgs/Debug debug

LatencyStats latency_stats

common_msgs/LocalizeState localize_state

# ebs temporary stop request: true request
bool temporary_stop_request
bool epb_request