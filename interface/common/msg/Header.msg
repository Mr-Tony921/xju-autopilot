# Message publishing time in seconds
float64 timestamp_sec

# Module name
string module_name

# Sequence number for each message. Each module maintains its own counter for
# sequence_num, always starting from 1 on boot.
uint32 sequence_num

# Lidar Sensor timestamp for nano-second.
uint64 lidar_timestamp

# Camera Sensor timestamp for nano-second.
uint64 camera_timestamp

# Radar Sensor timestamp for nano-second.
uint64 radar_timestamp

# data version
uint32 version

StatusPb status

string frame_id