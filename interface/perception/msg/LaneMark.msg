# LaneMarkingType, 车道线类型 
int32 type 0

# LaneMarkingColor, 车道线颜色
uint8 COLOR_NONE = 0
uint8 COLOR_YELLOW = 1
uint8 COLOR_WHITE = 2
uint8 COLOR_BLUE = 3
uint8 COLOR_ORANGE = 4

# LaneMarkingColor, 车道线颜色
uint8 color 0

# LaneMarkingQuality, 车道线质量
uint8 quality 2

#      ^x
#      |
#      |
# y<---o
# y = c3*x^3 + c2*x^2 + c1*x + c0
CubicCurve curve_vehicle_coord             # 车身坐标系下的车道线
common_msgs/Point3D[] pts_vehicle_coord    # 车身坐标系下的车道线点集

# o---->x
# |
# |
# ^y
# x = c3*y^3 + c2*y^2 + c1*y + c0
CubicCurve curve_image_coord               # 图像坐标系下的车道线
common_msgs/Point3D[] pts_image_coord      # 图像坐标系下的车道线点集
    
# 车道线id
int32 id

# LaneMarkingQualityStatus, Lane quality status
uint8 NORMAL = 0                 # lane line is good
uint8 ERROR_POSE = 1             # lane line is bad because input pose error
uint8 ERROR_LENGTH = 2           # lane line is bad because length is short
uint8 ERROR_POINT_NUM = 3        # lane line is bad because observe points num is little
uint8 ERROR_JUMP = 4             # lane line is bad because the jump distance between last status is big
uint8 ERROR_CURVATURE = 5        # lane line is bad because the radius of curvature is small
uint8 ERROR_POSITION = 6         # lane line is bad because intersection or spread of two lane
uint8 ERROR_CONFIDENCE = 7       # lane line is bad because confidence is low
uint8 ERROR_OUTLIER_RATIO = 8    # lane line is bad because outliter ratio is high
uint8 ERROR_OTHER = 9            # reserve

# LaneMarkingQualityStatus, 车道线质量的具体状态
uint8 quality_status

# LaneMarkingSourceType 
uint8 PERCEPTION = 0             # lane line is from PERCEPTION
uint8 HDMAP = 1                  # lane line is from HDMAP
uint8 FUSION = 2                 # part from HDMAP, part from PERCEPTION
uint8 OTHER = 3                  # reserve

# LaneMarkingSourceType, 车道线输出的来源
uint8 source_type

# LaneMarkingPosition
int8 POSITION_UNKNOWN = 0        # unknow position id
int8 CURB_LEFT = 5               # lane curb line on the left side
int8 FOURTH_LEFT = 4
int8 THIRD_LEFT = 3              # lane line on the left side next to adjacent lane
int8 ADJACENT_LEFT = 2           # lane line on the left side next to ego lane
int8 EGO_LEFT = 1                # left lane line of the ego lane
int8 EGO_RIGHT = -1              # right lane line of the ego lane
int8 ADJACENT_RIGHT = -2         # lane line on the right side next to ego lane
int8 THIRD_RIGHT = -3            # lane line on the right side next to adjacent lane
int8 FOURTH_RIGHT = -4
int8 CURB_RIGHT = -5             # lane curb line on the right side

# LaneMarkingPosition
int8 position

# 分段给的车道线类型
LaneMarkingTypeSection[] linetype_sections