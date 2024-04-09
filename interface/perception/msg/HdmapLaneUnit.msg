# 高精车道一个小段的i
int32 lane_id   

# 高精车道_link_i
int32 link_id   

# 左车道线
HdmapLineMark left_linemarking  

# 右车道线
HdmapLineMark right_linemarking   

# 该小段高精车道最高限速信息
float64 max_speed_limit

# 该小段高精车道最低限速信息
float64 min_speed_limit

# 该小段高精车道的曲率点
int32[] curvature_set

# 该小段高精车道的lane_type 
int32 lane_type   

# 该小段高精车道的restricted_lane_type
uint8 restricted_lane_type  

# 该小段高精车道的曲率点偏移量， 与曲率点一一对应，已转到车身系下，单位为m
float64[] curvature_set_offset               