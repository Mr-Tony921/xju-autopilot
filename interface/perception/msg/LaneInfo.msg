# 车道id， 该id与上面HdmapLaneUnit中的lane_id不同，这里是EM融合出车道的lane_id
int32 lane_id 
int32 leftline_id   # 该车道左车道线id 
int32 rightline_id  # 该车道右车道线id 


CubicCurve curve_center             # 车身坐标系下的车道线中心线
common_msgs/Point3D[] pts_center    # 车身坐标系下的车道线中心线点集


# 该车道前方15m处所在的HdmapLaneUnit的lane_type
int32 lane_type 

# 该车道前方15m处所在的HdmapLaneUnit的最高限速信息
float64 max_speed_limit 

# 该车道前方15m处所在的HdmapLaneUnit的最低限速信息
float64 min_speed_limit  

# 车道线宽
float64 width    

# 该车道对应的高精车道数据
HdmapLaneUnit[] hdmap_lane_data  

# 所有汇入该车道的车道id集
int32[] incoming_lane_ids 

# 该车道前方15m处所在的HdmapLaneUnit的restricted_lane_type
uint8 restricted_lane_type

# LaneMarkingQuality
uint8 leftline_quality
uint8 rightline_quality

# 车道是否消亡 0 没有消亡 1 即将消亡
bool is_lane_ending
# 车道消亡距离 unit m
float64 lane_end_length

# LaneOrder, 车道顺序, 从右向左
uint8 UNKOWN_ORDER = 0
uint8 FIRST_LANE = 1
uint8 SECOND_LANE = 2
uint8 THIRD_LANE = 3
uint8 FORTH_LANE = 4

uint8 lane_order