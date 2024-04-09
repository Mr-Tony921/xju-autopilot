# EpbStatus
uint8 EPB_UNKNOWN = 0
uint8 RELEASE = 1
uint8 APPLY = 2
uint8 APPLYING = 3
uint8 RELEASING = 4

common_msgs/Header header

float64 deceleration_command #减速命令
float64 deceleration_actual  #实际减速

float64 pedal_command #踏板命令
float64 pedal_actual  #实际踏板

bool ebs_override #ebs 重写
bool epb_override #epb 重写

bool ebs_fault #ebs 错误
bool epb_fault #epb 错误

uint8 epb_status #epb 状态