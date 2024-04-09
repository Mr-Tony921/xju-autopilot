common_msgs/Header header
# target torque in Nm， 轮端扭矩，即车辆驱动扭矩，非电机直接输出扭矩
float64 wheel_torque
# target decelerate in m/s2
float64 decelerate
# target steering angle, in deg
float64 steering
# target non-directional steering rate, in deg/s
float64 steering_rate

float64 acceleration
float64 speed

# parking brake engage. true: engaged
bool parking_brake
uint8 turn_signal
uint8 gear_position
bool emergency_light
uint16 error_code

float64 vehicle_mass

Debug debug
chart_msgs/Debug visualization_debug

# true: FSM state is control 
bool wheel_torque_isvalid
bool decelerate_isvalid
bool steering_isvalid

bool standstill_brake