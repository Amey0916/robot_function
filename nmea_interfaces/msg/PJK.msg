std_msgs/Header header

string msg_type           # 消息类型
float64 utc_time          # UTC时间，格式为hhmmss.sss
int32 utc_date            # UTC日期，DDMMYY格式
float64 x                 # X坐标，单位：米
string x_dir              # X坐标方向,N/S
float64 y                 # Y坐标，单位：米
string y_dir              # Y坐标方向,E/W
uint16 pos_status         # 解状态， 0：无效解，1：单点定位解，2：伪距差分，3：固定解，4：浮动解
uint16 satellites_count   # 使用卫星数量
float64 hdop              # HDOP-水平精度因子，0.5到99.9，一般认为HDOP越小，质量越好
float64 altitude          # 海拔高，-9999.9到9999.9米
string altitude_unit      # 高程单位，M指单位米

