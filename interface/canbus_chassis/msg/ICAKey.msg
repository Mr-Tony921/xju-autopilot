uint8       TURN_LIGHT_LEVEL_DEFAULT                        = 0
uint8       TURN_LIGHT_LEVEL_TURN_LEFT                      = 1
uint8       TURN_LIGHT_LEVEL_TURN_RIGHT                     = 2
uint8       TURN_LIGHT_LEVEL_INVALID                        = 3

uint8       DRIVING_MODE_MANUAL                             = 0
uint8       DRIVING_MODE_AUTOMODE_FULL                      = 1
uint8       DRIVING_MODE_AUTOMODE_STEER                     = 2
uint8       DRIVING_MODE_AUTOMODE_SPEED                     = 3
uint8       DRIVING_MODE_MCU_FAIL                           = 4 # MCU进入失败，一般是不符合进入条件
uint8       DRIVING_MODE_MCU_QUIT                           = 5 # 自驾过程中不符合某种条件，MCU退出自驾
uint8       DRIVING_MODE_VEHICLE_QUIT                       = 6 # 自驾过程中，车辆本身的原因，MCU退出自驾
uint8       DRIVING_MODE_MCU_QUITING                        = 7 # MCU退出自驾中，短暂状态
uint8       DRIVING_MODE_MCU_UNKNOMN                        = 8 # MCU状态未知

uint8       SWITCH_OFF                                      = 0 
uint8       SWITCH_ON                                       = 1 


uint8       AUTO_LANE_CHANGE_CONFIRM_NOT_CONFIRMATION       = 0
uint8       AUTO_LANE_CHANGE_CONFIRM_CONFIRMATION           = 1

uint8       REMINDER_SWITCH_REMINDER_OFF                    = 0
uint8       REMINDER_SWITCH_BEEP                            = 1
uint8       REMINDER_SWITCH_SOUND                           = 2
uint8       REMINDER_SWITCH_RESERVERD                       = 3

uint8       LANE_CHANGE_STYLE_LOW_SENSITIVITY               = 0
uint8       LANE_CHANGE_STYLE_NORMAL                        = 1
uint8       LANE_CHANGE_STYLE_HIGH_SENSITIVITY              = 2
uint8       LANE_CHANGE_STYLE_RESERVED                      = 3

uint8       HNP_WORK_STATUS_REJECT                          = 0
uint8       HNP_WORK_STATUS_READY                           = 1
uint8       HNP_WORK_STATUS_CONTROL                         = 2
uint8       HNP_WORK_STATUS_FAILURE                         = 10

uint8       HWA_EXIT_REASON_NONE                            = 0
uint8       HWA_EXIT_REASON_BRAKE_PEDAL_SWITCH              = 1
uint8       HWA_EXIT_REASON_EPB_PULL                        = 2
uint8       HWA_EXIT_REASON_GEAR_NOT_D                      = 3
uint8       HWA_EXIT_REASON_SPEED_LESS_THAN_ZERO            = 4
uint8       HWA_EXIT_REASON_ASSOCIATIVE_SUBSYSTEM_ERROR     = 5
uint8       HWA_EXIT_REASON_STEER_ANGLE_MORE_THAN_360       = 6
uint8       HWA_EXIT_REASON_ICA_CANCLE                      = 7
uint8       HWA_EXIT_REASON_ODD_FAILED                      = 8
uint8       HWA_EXIT_REASON_AEB_ACTIVE                      = 9
uint8       HWA_EXIT_REASON_SPEED_MORE_THAN_100             = 10
uint8       HWA_EXIT_REASON_VDCU_HANDSHAKE_FAILED           = 11
uint8       HWA_EXIT_REASON_VDC_ACTIVE                      = 12
uint8       HWA_EXIT_REASON_ABS_ACTIVE                      = 13
uint8       HWA_EXIT_REASON_ASR_ACTIVE                      = 14
uint8       HWA_EXIT_REASON_HANDS_OFF                       = 15
uint8       HWA_EXIT_REASON_DRIVER_FATIGUE_WARNING          = 16
uint8       HWA_EXIT_REASON_DRIVER_ATTENTION_WARNING        = 17
uint8       HWA_EXIT_REASON_DRIVER_SEAT_BELT_OPEN           = 18
uint8       HWA_EXIT_REASON_DRIVER_DOOR_OPEN                = 19
uint8       HWA_EXIT_REASON_SYSTEM_ERROR                    = 20


bool        cruise_switch                                           # 开启ica
bool        cruise_cancel                                           # 取消ica
bool        speed_up_and_res                                        # 速度加、resume
bool        speed_down                                              # 速度减
bool        cruise_distance_increase                                # 增加巡航距离
bool        cruise_distance_decrease                                # 减少巡航距离
uint8       turn_light_level                                        # 转向灯
uint8       automode_status                                         # 自动驾驶状态
uint8       hna_switch                                              # HNA功能开关
uint8       lane_change_auxiliary_switch                            # 变道辅助开关
uint8       auto_lane_change_confirmation_switch                    # 自动变道确认开关
uint8       lane_change_reminder_switch                             # 变道提醒开关
uint8       lane_change_style_setting                               # 变道风格设置
uint8       hnp_work_status                                         # HNP工作状态
uint8       hwa_exits_reason                                        # hwa激活状态退出原因


