uint64 sequence                                        #  序列号
uint64 sys_time_us                                     #  时间戳
string[] localize_error_info                           #  定位故障描述
common_msgs/Quaternion rotation                        #  四元数
common_msgs/Vector3D euler_ypr                         #  欧拉角, yaw，pitch，roll（弧度）
common_msgs/Vector3D position                          #  位置结果, x, y, z(m)
common_msgs/Vector3D velocity                          #  局部坐标系下的速度分量（m/s）
common_msgs/Vector3D velocity_in_vehicle               #  车身坐标系下的速度分量（m/s）
common_msgs/Vector3D angular_velocity_in_vehicle       #  车辆的角速度（rad/s）
common_msgs/Vector3D linear_acceleration_in_vehicle    #  车辆的加速度（m/s2） 
uint64 veh_fuse_vel_scale
