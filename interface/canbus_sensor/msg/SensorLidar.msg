std_msgs/Header header
bool update        #是否更新
float64 longitude   #经度-84坐标系  （单位 deg）
float64 lattitude   #纬度  （单位 deg）
float64 height      #高度  （单位 m）
float64 roll        #横滚角  （单位 deg）
float64 pitch       #府仰角  （单位 deg）
float64 yaw         #航向角  （单位 deg）
float64 xg          #UTM坐标下xg  （单位 m）
float64 yg          #UTM坐标下yg  （单位 m）
float64 zg          #UTM坐标下zg  （单位 m）
float64 xg_std      #xg_标准差
float64 yg_std      #yg_标准差
float64 zg_std      #zg_标准差
float64 roll_std    #roll_标准差
float64 pitch_std   #pitch_标准差
float64 yaw_std     #yaw_标准差
float64 result      #lidar匹配定位分值
