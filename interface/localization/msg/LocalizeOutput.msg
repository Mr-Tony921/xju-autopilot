# GlobalLocalizeState 
uint8 AHRS_NOT_READY_FOR_INIT           = 0
uint8 AHRS_INITIALIZING                 = 1
uint8 AHRS_WAIT_IMU                     = 2
uint8 AHRS_WAIT_VEHICLE_SPEED           = 3
uint8 AHRS_WAIT_GPS                     = 4
uint8 AHRS_WAIT_GPS_FIX                 = 5
uint8 AHRS_SENSOR_READY                 = 6
uint8 AHRS_FUSE_GPS_POS_CONVERGED       = 7
uint8 AHRS_FUSE_MATCH_POS_CONVERGED     = 8
uint8 AHRS_FUSE_MATCH_HEADING_CONVERGED = 9
uint8 AHRS_FUSE_ALL_GOOD                = 10

# LocalLocalizeState
uint8 UNKNOWN_L                 = 0  # 默认
uint8 WAIT_L                    = 1  # 等待
uint8 INITIALIZING_L            = 2  # 初始化
uint8 TRACKING_L                = 6  # 定位正常输出
uint8 ERROR_L                   = 4  # 故障

common_msgs/Header header

uint64 sequence       # 序列号
uint64 sys_time_us    #  系统时间 (单位 us)
float64 utc_time_sec  # UTC时间 (单位 sec)

# 全局定位信息
uint8 global_localize_state     #  全局定位状态, in vehicle
GlobalLocalizeResult global_localize_result   #  全局定位结果, in vehicle

# 局部定位信息
uint8 local_localize_state       #  局部定位状态, in vehicle
LocalLocalizeResult local_localize_result     #  局部定位结果, in vehicle
    
# 局部到全局转换信息
Transformd global_to_local                    #  Global to Local transform
