# TurnSignal
uint8 TURN_NONE = 0
uint8 TURN_LEFT = 1
uint8 TURN_RIGHT = 2

# Direction
uint8 FORWARD = 0
uint8 BACKWARD = 1

# GearPosition
uint8 NEUTRAL = 0
uint8 DRIVE = 1
uint8 REVERSE = 2
uint8 PARKING = 3
uint8 LOW = 4
uint8 INVALID = 5
uint8 NONE = 6

# DrivingMode
uint8 COMPLETE_MANUAL = 0 # human drive
uint8 COMPLETE_AUTO_DRIVE = 1
uint8 AUTO_STEER_ONLY = 2  # only steer
uint8 AUTO_SPEED_ONLY = 3 # include throttle and brake
# security mode when manual intervention happens, only response status
uint8 EMERGENCY_MODE = 4