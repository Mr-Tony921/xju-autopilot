EmObstacle perception_obstacle

PredictionTrajectory[] prediction_trajectory

# ObstacleIntent
uint8 UNKNOWN = 0
uint8 STOP = 1
uint8 STATIONARY = 2
uint8 MOVING = 3
uint8 CHANGE_LANE = 4
uint8 LOW_ACCELERATION = 5
uint8 HIGH_ACCELERATION = 6
uint8 LOW_DECELERATION = 7
uint8 HIGH_DECELERATION = 8

# estimated obstacle intent
uint8 intent 0

bool is_static
bool is_caution