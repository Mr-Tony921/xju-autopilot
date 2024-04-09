#sensor_msgs/PointCloud2 point_cloud
common_msgs/Point3D[] ob_polygon

float64 x_c 
float64 y_c
float64 z_c 

float64 center_x 
float64 center_y 
float64 center_z 

string object_type

float64 confidence

geometry_msgs/Vector3 heading
float64 yaw

float64 l
float64 w 
float64 h
#目标真实长宽高及中心点坐标
float64 rotated_rect_w         #旋转目标宽（横向）
float64 rotated_rect_l         #旋转目标长（纵向）
float64 rotated_rect_h         #旋转目标高
float64 rotated_rect_center_x  #旋转目标中心点坐标x
float64 rotated_rect_center_y  #旋转目标中心点坐标y
float64 rotated_rect_center_z  #旋转目标中心点坐标z
#时间戳
uint64 time_stamp
