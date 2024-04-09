# SpeedDirection
uint8 FORWARD = 0
uint8 REVERSE = 1
uint8 STANDSTILL = 2

common_msgs/Header header

bool isvalid

# unit m/s
float64 speed

# from ebs
float64 acceleration 
uint8 direction