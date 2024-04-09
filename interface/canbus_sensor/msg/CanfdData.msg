std_msgs/Header header
uint64 dev_msg_timestamp  # TC397接收外设数据到来的时间点，精确到毫秒
uint64 soc_msg_timestamp  # SOC发送数据到SOC的时间点，精确到毫秒
uint32 canid              # 11-bit or 29-bit CAN ID 
uint8 dlc                 # Data length:  (0 to 64 bytes)
uint8[] data              # Data payload, up to 64 bytes
