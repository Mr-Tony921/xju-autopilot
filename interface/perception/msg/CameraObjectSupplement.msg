ObstacleStatus is_onroad
ObstacleStatus is_occluded
ObstacleStatus is_truncated
ObstacleStatus is_sprinkler
BarrierType barrier_type
SpecialVehicleType sv_type
SpecialVehicleStatus sv_status

ObstacleStatus is_3d_uncertain

ObstacleStatus is_lane_refined

SubtypeSizeRelation subtype_size_relation

LocatedInLaneType located_in_lane_type

float64 onroad_prob
float64 occluded_prob
float64 truncated_prob
float64 sprinkler_prob
float64 barrier_prob
float64 is_3d_uncertain_prob

float64 height_ratio_pred

common_msgs/Point2D projected_center

bool on_use
bool only_2d

string sensor_name
float64 timestamp 

uint32 frame_id

uint32 global_frame_id

Box2D box

CameraRawObject raw_object

string camera_fusion_choose_sensor_name

Box2D projected_box
int32 local_track_id

float64[] pts8
Box2D front_box
Box2D back_box
float64[] object_feature

float64 alpha
float64 truncated_horizontal
float64 truncated_vertical

common_msgs/Vector3D local_center
VisualObjectType visual_type 
float64[] visual_type_probs

uint32 area_id 

float64[] visible_ratios

float64[] cut_off_ratios 

OccludedStatus occluded_status 

uint32[] afore_obstacles

ObstacleLaneType obstacle_lane_type
AbnormalStatus abnormal_status 

CameraObjectTrackState track_state 
Vector2D[] ground_touching_points 
