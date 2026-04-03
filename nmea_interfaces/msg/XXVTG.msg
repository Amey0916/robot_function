std_msgs/Header header

string msg_type                 # 消息类型
float32 movement_angle_true_north  # 运动角度，000 - 359，以真北为参考基准的地面航向，前面的0也将被传输
string true_north_ref           # T=真北参照系
float32 movement_angle_mag_north   # 运动角度，000 - 359，以磁北为参考基准的地面航向，前面的0也将被传输
string magnetic_north_ref       # M=磁北参照系
float32 horizontal_move_speed_knots   # 水平运动速度，000.0~999.9节，前面的0也将被传输
string speed_knots              # N=节，Knots
float32 horizontal_move_speed_kmh   # 水平运动速度，0000.0~1851.8Km/h，前面的0也将被传输
string speed_kmh                # K=公里/时，km/h
string mode_ind                 # 模式指示,仅NMEA0183 v3.00输出，A=自主定位，D=差分，E=估算，N=数据无效

