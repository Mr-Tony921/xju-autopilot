std_msgs/Header header
bool update        #是否更新
uint32 timestamp_sec #UNIX时间
uint32 timestamp_nsec #UNIX时间
float64 longitude   #经度-84坐标系(单位 deg)
float64 lattitude   #纬度 (单位 deg)
float64 height      #高度 (单位 m)
float64 heading     #双天线航向角 （单位 deg）
float64 velocity    #地面2D速度 （单位 m/s）
float64 up_velocity #天速       （单位 m/s）
float64 velocity_n  #ned速度n分量（单位 m/s）
float64 velocity_e  #ned速度e分量（单位 m/s）
float64 velocity_d  #ned速度d分量（单位 m/s）
float64 track_angle #航迹角
float64 acc_ver     # 水平定位精度
float64 acc_hor     # 垂直定位精度
float64 acc_speed   # 速度精度
float64 acc_head    # 方向精度
float64 hdop        
float64 diff_age    #差分零期
float64 base_length #基线长度
float64 heading_std #航向角标准差
float64 xg          #UTM坐标下xg
float64 yg          #UTM坐标下yg
float64 zg          #UTM坐标下zg
uint8 status       #GPS状态 0:invalid , 1:单点定位，2：差分定位，4：固定解（fix），5：浮点解（float）
uint8 satenum      #卫星颗数
int32 err_code     #错误码 （1：账号连接中断；2：服务鉴权异常；3：通讯网络异常；4：GGA发送错误）
uint8 is_heading_valid #航向角是否有效