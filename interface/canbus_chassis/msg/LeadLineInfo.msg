uint8        DIRECTION_NO_DISPLAY   = 0         # 无方向
uint8        FRONT                  = 1         # 前
uint8        LEFT                   = 2         # 左
uint8        RIGHT                  = 3         # 右

uint8       COLOR_NO_DISPLAY        = 0         # 无颜色
uint8       RED                     = 1         # 红色
uint8       GREEN                   = 2         # 绿色
uint8       GRAY                    = 3         # 灰色
uint8       WHITE                   = 4         # 白色

uint8       direction                           # 引导线方向
uint8       color                               # 引导线颜色
float32     c0                                  # HNA开启时引导线五阶方程常数项
float32     c1                                  # HNA开启时引导线五阶方程一次项
float32     c2                                  # HNA开启时引导线五阶方程二次项
float32     c3                                  # HNA开启时引导线五阶方程三次项
float32     c4                                  # HNA开启时引导线五阶方程四次项
float32     c5                                  # HNA开启时引导线五阶方程五次项
uint8       cur_direction                       # 当前方向
uint8       cur_color                           # 当前颜色
float32     cur_length                          # 当前颜色






