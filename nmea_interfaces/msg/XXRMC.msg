std_msgs/Header header

string msg_type           # 消息类型
float64 utc_time          # UTC时间，hhmmss.sss格式
string status             # 状态，A=定位，V=未定位。当给GPS复位时为V，不输出速度，角度，时间数据
float64 latitude          # 纬度ddmm.mmmm，度分格式
string latitude_hemi      # 纬度N或S
float64 longitude         # 经度dddmm.mmmm，度分格式
string longitude_hemi     # 经度E或W
float32 speed_knots       # 速度，节，Knots
float32 azimuth           # 方位角，度 Azimuth/bearing
int32 utc_date            # UTC日期，DDMMYY格式
float32 mag_var           # 磁偏角，000-180度
string mag_var_dir        # 磁偏角方向，E或W
string mode_ind           # 模式指示,仅NMEA0183 v3.00输出，A=自主定位，D=差分，E=估算，N=数据无效

