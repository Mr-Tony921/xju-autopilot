common_msgs/Header header

float32      lgtd_accl          #IMU纵向加速度
float32      lat_accl           #IMU横向加速度
float32      vert_accl          #IMU垂向加速度
float32      roll_rate          #IMU侧倾角速度
float32      ptch_rate          #IMU俯仰角速度
float32      yaw_rate           #IMU横摆角速度
bool         lgtd_accl_vld      #IMU纵向加速度有效性
bool         lat_accl_vld       #IMU横向加速度有效性
bool         vert_accl_vld      #IMU垂向加速度有效性
bool         roll_rate_vld      #IMU侧倾角速度有效性
bool         ptch_rate_vld      #IMU俯仰角速度有效性
bool         raw_rate_vld       #IMU横摆角速度有效性
float64      longi             #经度
float64      lati              #纬度
