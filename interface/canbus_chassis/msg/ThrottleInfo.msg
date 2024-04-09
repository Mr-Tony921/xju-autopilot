common_msgs/Header header

# throttle pedal percentage 0 ~ 100.0
float64 pedal_output            
float64 pedal_command   

bool motor_torque_vaild
# unit Nm， 轮端扭矩，即车辆驱动扭矩，非电机直接输出扭矩
float64 motor_torque_command
float64 motor_torque_actual 

bool throttle_override 
bool throttle_fault  

# unit rpm
float64 motor_speed 
float64 veh_max_drv_tq 
float64 veh_max_rgn_tq