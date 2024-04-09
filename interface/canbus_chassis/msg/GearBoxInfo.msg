# GearBoxPosition
uint8 GEAR_UNKNOWN = 0
uint8 GEAR_P = 1
uint8 GEAR_R = 2
uint8 GEAR_N = 3
uint8 GEAR_D = 4

common_msgs/Header header

# GearBoxPosition
uint8 gearbox_command #挡位命令
uint8 gearbox_actual #实际挡位

bool gearbox_override #挡位重写
bool gearbox_fault #挡位错误

bool front_gearbox_shift_state #
bool rear_gearbox_shift_state #