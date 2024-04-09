# sensor timestamp in seconds(main sensor timestamp)
float64 timestamp

uint32 id                      # obstacle ID.

# obstacle position in the vehicle coordinate system.
common_msgs/Point3D position

# obstacle velocity in the vehicle coordinate system.
common_msgs/Point3D velocity   

# obstacle acceleration
common_msgs/Point3D acceleration 

# position covariance which is a row-majored 3x3 matrix
float64[] position_covariance 

# velocity covariance which is a row-majored 3x3 matrix
float64[] velocity_covariance 

# acceleration covariance which is a row-majored 3x3 matrix
float64[] acceleration_covariance 

# heading in the vehicle coordinate system.
float64 theta 

# Size of obstacle bounding box.
float64 length   # obstacle length.

float64 width    # obstacle width.

float64 height   # obstacle height.

# Obstacle Type 
uint8 UNKNOWN = 0
uint8 UNKNOWN_MOVABLE = 1
uint8 UNKNOWN_UNMOVABLE = 2
uint8 PEDESTRIAN = 3         # Pedestrian, usually determined by moving behaviour.
uint8 BICYCLE = 4            # bike, motor bike
uint8 VEHICLE = 5            # Passenger car or truck.

# obstacle type
uint8 type   

# SubType
uint8 ST_UNKNOWN = 0
uint8 ST_UNKNOWN_MOVABLE = 1
uint8 ST_UNKNOWN_UNMOVABLE = 2
uint8 ST_CAR = 3
uint8 ST_VAN = 4
uint8 ST_TRUCK = 5
uint8 ST_BUS = 6
uint8 ST_CYCLIST = 7
uint8 ST_MOTORCYCLIST = 8
uint8 ST_TRICYCLIST = 9
uint8 ST_PEDESTRIAN = 10
uint8 ST_TRAFFICCONE = 11

# animals
uint8 ST_CAT = 12
uint8 ST_DOG = 13

# barriers
uint8 ST_TRAFFICCONE_DOWN = 14          # deprecated, merge into BARRIER_CONE (2020-10-16)
uint8 ST_SAFETY_TRIANGLE = 15           # deprecated, rename as BARRIER_TRIANGLE (2020-10-16)
uint8 ST_SECURITY_ARM_BARRIER = 16

uint8 ST_VEGETATION = 17
uint8 ST_FENCE = 18

uint8 ST_BARRIER_CONE = 19              # 普通锥筒
uint8 ST_BARRIER_DELINEATOR = 20        # 交通桩（金属柱、水泥柱）
uint8 ST_BARRIER_WATER = 21             # 水马
uint8 ST_BARRIER_BUCKET = 22            # 防撞桶
uint8 ST_BARRIER_CEMENT = 23            # 水泥隔离墩
uint8 ST_BARRIER_STONE = 24             # 石墩：（圆、椭圆、球形）
uint8 ST_BARRIER_NO_PARKING_SIGN = 25   # 停车指示牌（金属、塑料）
uint8 ST_BARRIER_TEMPORARY_SIGN = 26    # 临时交通指示牌：方形、圆形
uint8 ST_BARRIER_TRIANGLE = 27          # 三角警示牌
uint8 ST_BARRIER_WHEEL_STOPPER = 28     # 车位阻轮器
uint8 ST_BARRIER_PARKING_LOCK = 29      # 车位锁

uint8 ST_VAN_MICRO = 30
uint8 ST_VAN_MEDIUM = 31

# obstacle sub_measurementstype
uint8 sub_type

# size = 0:ignore, 1:in one lane, 2:in two lane, all id: can not sure but need use
uint32[] lane_ids

# obstacle 3d corner points.
common_msgs/Point3D[] polygon_point 

# duration of an obstacle since detection in s.
float64 tracking_time

# 关联传感器
int32 fusion_state  

# sensor measurements
SensorMeasurement[] measurements  
  
ObsTrajectory trajectory 