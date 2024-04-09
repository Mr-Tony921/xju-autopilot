# interface

## 1. topic name

| module              | topic name               | message                                   |
| ------------------- | ------------------------ | ----------------------------------------- |
| planning            | "/pnc/planning"          | planning_msgs::msg::Planning              |
| control             | "/pnc/control"           | control_msgs::msg::Control                |
| localization        | "/localization/output"   | localization_msgs::msg::LocalizeOutput    |
| map                 | "/map/routing"           | routing_msgs::msg::RoutingInfo            |
| prediction          | "/em/prediction"         | perception_msgs::msg::PredictionObstacles |
| emLanes             | "/em/emlanes"            | perception_msgs::msg::EmLanes             |
| chassis brake       | "/chassis/brake"         | chassis_msgs::msg::BrakeInfo              |
| chassis gear        | "/chassis/gear"          | chassis_msgs::msg::GearBoxInfo            |
| chassis throttle    | "/chassis/throttle"      | chassis_msgs::msg::ThrottleInfo           |
| chassis steer       | "/chassis/steer"         | chassis_msgs::msg::SteerInfo              |
| chassis speed       | "/chassis/vehicle_speed" | chassis_msgs::msg::CarSpeedInfo           |
| chassis wheelspeed  | "/chassis/wheel_speed"   | chassis_msgs::msg::WheelSpeedInfo         |
| chassis vehiclemass | "/chassis/vehicle_mass"  | chassis_msgs::msg::CarMassInfo            |
| emObstacles         | "/em/obstacles"          | perception_msgs::msg::EmObstacles         |

## 2 Field types

| Type name |      C++       |   Python    |   DDS type   |
| --------- | -------------- |-------------|--------------|
| bool      | bool           |-------------|--------------|        
| byte      | uint8_t        |-------------|--------------|            
| char      | char           |-------------|--------------|
| float32   | float          |-------------|--------------| 
| float64   | double         |-------------|--------------|
| int8      | int8_t         |-------------|--------------|
| uint8     | uint8_t        |-------------|--------------|
| int16     | int16_t        |-------------|--------------|
| uint16    | uint16_t       |-------------|--------------|
| int32     | int32_t        |-------------|--------------|
| uint32    | uint32_t       |-------------|--------------|
| int64     | int64_t        |-------------|--------------|
| uint64    | uint64_t       |-------------|--------------|
| string    | std::string    |-------------|--------------|
| wstring   | std::u16string |-------------|--------------|