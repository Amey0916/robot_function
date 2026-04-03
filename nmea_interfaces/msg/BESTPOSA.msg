std_msgs/Header header

string msg_type       # 消息类型
uint8 sol_status      # 结算状态
uint8 pos_status      # 定位状态
float64 latitude      # 纬度
float64 longitude     # 经度
float64 altitude      # 海拔高
float64 geoid_height  # 大地水准面高度异常差值
string datum_id       # 坐标系统
float64 lat_std_dev   # 纬度标准差
float64 lon_std_dev   # 经度标准差
float64 hgt_std_dev   # 高度标准差
string stn_id         # 基准值ID
float64 diff_age      # 差分龄期（以秒计）
float64 sol_age       # 解算时间（以秒计）
uint16 svs            # 跟踪到的卫星数
uint16 sol_svs        # 参与 RTK 解算的卫星数
uint16 l1_sol_svs     # L1 参与 PVT 解算的卫星数
uint16 l1_l2_sol_svs # L1 和 L2 参与 PVT 解算的卫星数
uint16 reserved1      # 预留
uint16 ext_sol_status # 扩展解算状态
uint16 reserved2      # 预留
uint16 sol_sigs       # 参与解算的信号

