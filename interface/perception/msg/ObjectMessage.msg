uint32 id

# obstacle 3d corner points.
common_msgs/Point3D[] polygon

common_msgs/Vector3D direction

float64 theta

float64 theta_variance

common_msgs/Vector3D center 

float64[] center_uncertainty

common_msgs/Vector3D size 

common_msgs/Vector3D size_variance 

common_msgs/Vector3D anchor_point 

ObjectType type 

float64[] type_probs

ObjectSubType sub_type

float64[] sub_type_probs

ParsingType parse_type
float64 parse_type_probs

float64 confidence 

float64 existence_prob 

ConfidenceType confidence_type 

int32 track_id 

common_msgs/Vector3D velocity

float64[9] velocity_uncertainty

bool velocity_converged 

HeadingStatus heading_status

float64 velocity_confidence

common_msgs/Vector3D acceleration 

float64[9] acceleration_uncertainty

float64 tracking_time 

float64 latest_tracked_time 

MotionState motion_state 

CarLight car_light

RoadState road_state

ObjectTags object_tags

PolygonDType shadow_region

float64 dist_by_time_to_collision
float64 time_to_collision
LocatedInLaneType located_in_lane_type 
uint8 lane_id   
uint8[] lane_ids

LidarObjectSupplement lidar_supplement
RadarObjectSupplement radar_supplement
CameraObjectSupplement camera_supplement

#TrafficScenarioType[] traffic_scenario_types

#float64[][] traffic_scenario_type_probs

bool hmi_obstacle_render_flag