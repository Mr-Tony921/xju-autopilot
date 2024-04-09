uint8       TARGET_TYPE_UNKNOW              = 0     # 未知
uint8       TARGET_TYPE_CAR                 = 1     # 小车
uint8       TARGET_TYPE_TRUCK               = 2     # 卡车
uint8       TARGET_TYPE_BUS                 = 3     # 客车
uint8       TARGET_TYPE_MOTORCYCLE          = 4     # 摩托车
uint8       TARGET_TYPE_BICYCLE             = 5     # 自行车
uint8       TARGET_TYPE_PEDESTRIAN          = 6     # 行人
uint8       TARGET_TYPE_TRICYCLE            = 7     # 锥筒

uint8       LATERAL_DIRECTION_LEFT          = 0     # 左侧
uint8       LATERAL_DIRECTION_RIGHT         = 1     # 右侧

uint32      TARGET_NOT_DECTECTED            = 0     # 未探测
uint8       TARGET_DECTECTED                = 1     # 探测

uint8       detected
uint8       target_type
float32     lateral_dist
float32     longitudinal_dist
uint8       lateral_direction
uint32      id
float64     theta

