std_msgs/Header header

string msg_type           # 消息类型
uint8 sol_status          # 结算状态
uint8 pos_status          # 定位状态
float64 base_len          # 基线长，单位米
float64 yaw               # 方位角，0~360
float64 pitch             # 俯仰角，-90~90
float64 reserved1         # 预留
float64 yaw_std_dev       # 方位角标准差，单位度
float64 pitch_std_dev     # 俯仰角标准差，单位度
string stn_id             # 基准值ID
uint16 svs                # 跟踪到的卫星数
uint16 sol_svs            # 参与RTK解算的卫星数
uint16 obs                # 截止高度角以上的卫星数
uint16 l2_obs             # 截止高度角以上跟踪到L2的卫星数
uint16 reserved2          # 预留
uint16 ext_sol_status     # 扩展解算状态
uint16 reserved3          # 预留
uint16 sol_sigs           # 参与解算的信号

