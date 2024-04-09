# LaneType 
uint8 LAT_NORMAL = 0
uint8 LAT_ENTRY = 1
uint8 LAT_EXIT = 2
uint8 LAT_EMERGENCY = 3
uint8 LAT_ON_RAMP = 4
uint8 LAT_OFF_RAMP = 5
uint8 LAT_CONNECT_RAMP = 6
uint8 LAT_ACCELERATE = 7
uint8 LAT_DECELERATE = 8
uint8 LAT_DIVERSION = 9 # diversion zone



int32 left_sequence 0               # 车道定位序号
int32 right_sequence 0              # 车道定位序号
int64 lane_id 0                     # 车道定位id，高精地图一致
uint8 lane_type 0                   # 车道类型，高精地图一致
int32 lane_change_status 0          # 是否在变道过程中
int32 quality 0                     # 车道级定位状态
bool is_virtual_line false
 