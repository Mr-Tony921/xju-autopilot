# No error, returns on success.
uint16 OK = 0

# Control module error codes start from here.
uint16 CONTROL_ERROR = 1000
uint16 CONTROL_INIT_ERROR = 1001
uint16 CONTROL_COMPUTE_ERROR = 1002
uint16 CONTROL_ESTOP_ERROR = 1003
uint16 PERFECT_CONTROL_ERROR = 1004

# Canbus module error codes start from here.
uint16 CANBUS_ERROR = 2000
uint16 CAN_CLIENT_ERROR_BASE = 2100
uint16 CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED = 2101
uint16 CAN_CLIENT_ERROR_FRAME_NUM = 2102
uint16 CAN_CLIENT_ERROR_SEND_FAILED = 2103
uint16 CAN_CLIENT_ERROR_RECV_FAILED = 2104

# Localization module error codes start from here.
uint16 LOCALIZATION_ERROR = 3000
uint16 LOCALIZATION_ERROR_MSG = 3100
uint16 LOCALIZATION_ERROR_LIDAR = 3200
uint16 LOCALIZATION_ERROR_INTEG = 3300
uint16 LOCALIZATION_ERROR_GNSS = 3400

# Perception module error codes start from here.
uint16 PERCEPTION_ERROR = 4000
uint16 PERCEPTION_ERROR_TF = 4001
uint16 PERCEPTION_ERROR_PROCESS = 4002
uint16 PERCEPTION_FATAL = 4003
uint16 PERCEPTION_ERROR_NONE = 4004
uint16 PERCEPTION_ERROR_UNKNOWN = 4005

# Prediction module error codes start from here.
uint16 PREDICTION_ERROR = 5000

# Planning module error codes start from here
uint16 PLANNING_ERROR = 6000
uint16 PLANNING_ERROR_NOT_READY = 6001

# HDMap module error codes start from here
uint16 HDMAP_DATA_ERROR = 7000

# Routing module error codes
uint16 ROUTING_ERROR = 8000
uint16 ROUTING_ERROR_REQUEST = 8001
uint16 ROUTING_ERROR_RESPONSE = 8002
uint16 ROUTING_ERROR_NOT_READY = 8003

# Indicates an input has been exhausted.
uint16 END_OF_INPUT = 9000

# HTTP request error codes.
uint16 HTTP_LOGIC_ERROR = 10000
uint16 HTTP_RUNTIME_ERROR = 10001

# Relative Map error codes.
uint16 RELATIVE_MAP_ERROR = 11000  # general relative map error code
uint16 RELATIVE_MAP_NOT_READY = 11001

# Driver error codes.
uint16 DRIVER_ERROR_GNSS = 12000
uint16 DRIVER_ERROR_VELODYNE = 13000

# Storytelling error codes.
uint16 STORYTELLING_ERROR = 14000