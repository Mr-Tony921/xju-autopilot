SensorInfo sensor_info
float64 timestamp
ObjectMessage[] detected_objects
ObjectMessage[] tracked_objects
ObjectMessage[] objects
float64[16] sensor2world_pose  #matrix 4*4

#todo: add laneline message
#LaneLine lane_lines
#LaneLine tracked_lane_lines
#LaneLine fusion_lane_lines