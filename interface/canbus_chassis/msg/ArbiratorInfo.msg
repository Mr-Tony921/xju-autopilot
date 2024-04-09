

common_msgs/Header header
uint16      auto_mod_meq       #自动驾驶请求
uint8       tar_gear_req       #目标档位请求
float32     tar_dst            #目标停止距离
float32     tar_accl           #目标加速度
float32     accl_posn_req      #油门控制量请求
float32     brk_decel_req      #制动减速度请求
bool        epb_req            #EPB状态请求
uint8       lgtd_mod_req       #纵向模式控制    HU_Resp
bool        tar_dst_vld        #目标停止距离有效值
bool        tar_accl_vld       #目标加速度有效值
bool        accl_posn_req_vld  #油门控制量请求有效值
bool        decel_req_vld      #制动减速度请求有效值
bool        stand_still        #停车刹车指令
bool        brk_to_stop        #减速到停
bool        brk_drv_0ff        #起步释放刹车指令
uint8       ica_mod_req        #ICA状态       HU_Resp
int32       tar_mot_tq         #目标电机扭矩
bool        tar_mot_tq_vld     #目标电机扭矩有效性
float32     tar_steer_whl_ang  #目标方向盘转角
int16       tar_steer_whl_spd  #方向盘转速目标
float32     fr_whl_ang_req     #前轮转角请求
float32     tar_spd            #目标速度


