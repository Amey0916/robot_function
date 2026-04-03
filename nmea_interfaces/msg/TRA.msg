std_msgs/Header header

string msg_type             # 消息类型
float64 utc_time           # UTC时间，格式为hhmmss.sss
float64 yaw                # 方位角，0~360
float64 pitch              # 俯仰角，-90~+90
float64 roll               # 横滚角，-90~+90
uint16 pos_status          # 解状态， 0：无效解，1：单点定位解，2：伪距差分，4：固定解，5：浮动解
uint16 satellites_count    # 使用卫星数量
float32 differential_delay # 差分延时 
uint16 stnid               # 参考站号

