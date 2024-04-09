# OpenStatus 
uint8 OPEN = 0
uint8 CLOSE = 1
uint8 UNKNOWN = 2        

# ActiveStatus 
uint8 INACTIVE = 0
uint8 ACTIVE = 1
uint8 ACTIVE_UNKNOWN = 2

# WiperStatus 
uint8 WIPER_OFF = 0
uint8 WIPER_LOW = 1       # 低速
uint8 WIPER_HIGH = 2      # 高速
uint8 WIPER_MIST = 3      # 擦雾
uint8 WIPER_INTERVAL = 4  # 间歇
uint8 WIPER_AUTO = 5      # 自动(打开)
uint8 WIPER_UNKNOMN = 6

uint8 HNP_REJECT = 0
uint8 HNP_READY = 1 
uint8 HNP_CONTROL = 2
uint8 HNP_FAILURE = 10

common_msgs/Header header

# OpenStatus 
uint8 left_front_door
uint8 right_front_door
uint8 left_rearview_mirror
uint8 right_rearview_mirror

# ActiveStatus 
uint8 driver_seat 
uint8 passenger_seat 
uint8 driver_belt 
uint8 passenger_belt 

# WiperStatus 
uint8 wiper 6

LightInfo light_info

float64 vehicle_mass

uint8 hnp_work_status 0
