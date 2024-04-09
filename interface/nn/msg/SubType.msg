uint8    SUBTYPE_UNKNOWN             = 0
uint8    SUBTYPE_UNKNOWN_MOVABLE     = 1
uint8    SUBTYPE_UNKNOWN_UNMOVABLE   = 2
uint8    SUBTYPE_CAR                 = 3  # cam提供  lidar 提供
uint8    SUBTYPE_VAN                 = 4  # cam提供  lidar 提供
uint8    SUBTYPE_TRUCK               = 5  # cam提供  lidar 提供
uint8    SUBTYPE_BUS                 = 6  # cam提供  lidar 提供
uint8    SUBTYPE_CYCLIST             = 7  # cam提供  lidar 提供
uint8    SUBTYPE_MOTORCYCLIST        = 8  # cam提供  lidar 提供
uint8    SUBTYPE_TRICYCLIST          = 9  # cam提供  lidar 提供
uint8    SUBTYPE_PEDESTRIAN          = 10  # cam提供  lidar 提供
uint8    SUBTYPE_CAT                 = 11  # cam不提供  lidar 不提供
uint8    SUBTYPE_DOG                 = 12  # cam不提供  lidar 不提供
uint8    SUBTYPE_SECURITY_ARM_BARRIER = 13       # 升降杆（道闸）             #cam不提供  lidar 不提供
uint8    SUBTYPE_BARRIER_CONE        = 14               # 普通锥筒                  #cam提供    lidar  提供
uint8    SUBTYPE_BARRIER_DELINEATOR  = 15         # 交通桩（金属柱、水泥柱）     #cam提供   lidar 不提供
    # 水马                     #cam提供     lidar 不提供
uint8    SUBTYPE_BARRIER_WATER       = 16
    # 防撞桶                   #cam提供     lidar 不提供
uint8    SUBTYPE_BARRIER_BUCKET      = 17
uint8    SUBTYPE_BARRIER_CEMENT      = 18             # 水泥隔离墩                #cam提供    lidar 不提供
uint8    SUBTYPE_BARRIER_STONE       = 19              # 石墩：（圆、椭圆、球形）    #cam提供   lidar 不提供
uint8    SUBTYPE_BARRIER_NO_PARKING_SIGN = 20    # 停车指示牌（金属、塑料）    #cam提供   lidar 提供
uint8    SUBTYPE_BARRIER_TEMPORARY_SIGN = 21     # 临时交通指示牌：方形、圆形  #cam提供   lidar 提供
uint8    SUBTYPE_BARRIER_TRIANGLE        = 22           # 三角警示牌               #cam提供     lidar 提供
    # 车位阻轮器               #cam提供     lidar 不提供
uint8    SUBTYPE_BARRIER_WHEEL_STOPPER = 23
    # 车位锁                  #cam提供     lidar 不提供
uint8    SUBTYPE_BARRIER_PARKING_LOCK = 24
uint8    SUBTYPE_VEGETATION          = 25                 # 植被                    #cam不提供   lidar 不提供
uint8    SUBTYPE_FENCE               = 26                      # 栅栏                    #cam不提供   lidar 不提供
uint8    SUBTYPE_RESERVE1            = 27
uint8    SUBTYPE_RESERVE2            = 28
uint8    SUBTYPE_RESERVE3            = 29
uint8    SUBTYPE_VAN_MICRO           = 30
uint8    SUBTYPE_VAN_MEDIUM          = 31
    # SUBTYPE_MAX_OBJECT_TYPE = 32  TODO ???

uint8 subtype