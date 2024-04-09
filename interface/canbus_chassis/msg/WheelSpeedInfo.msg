# WheelDirection
uint8 FORWARD = 0
uint8 BACKWARD = 1
uint8 STANDSTILL = 2
uint8 INVALID = 3

common_msgs/Header header

bool front_left_valid     
float64 front_left       
uint8 front_left_direction 
int32 front_left_counter   

bool front_right_valid    
float64 front_right          
uint8 front_right_direction
int32 front_right_counter  

bool rear_left_valid      
float64 rear_left            
uint8 rear_left_direction  
int32 rear_left_counter    

bool rear_right_valid     
float64 rear_right           
uint8 rear_right_direction 
int32 rear_right_counter   

float64     front_left_speed
float64     front_right_speed
float64     rear_right_speed
float64     rear_left_speed 

float32 front_left_pressure         # 左前轮气压
float32 front_right_pressure        # 右前轮气压
float32 rear_left_pressure          # 左后轮气压
float32 rear_back_pressure          # 右后轮气压
