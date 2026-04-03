std_msgs/Header header

string msg_type               # 消息类型
uint16 item_count             # GSV语句的总数，1~3
uint16 item_sequence          # 本句GSV的编号，1~3
uint16 satellites_count       # 可见卫星的总数，00~12，前面的0也将被传输
uint16 prn_code               # PRN伪随机噪声码，前面的0也将被传输 GPS:1-32,Beidou:1-37,GLONASS:1-24,Galileo:1-36,...
float32 satellite_elevation   # 卫星仰角，00~90度，前面的0也将被传输
float32 satellite_azimuth     # 卫星方位角，000~359度，前面的0也将被传输
float32 signal_noise_ratio    # 信噪比，00~99dB，没有跟踪到卫星时为空，前面的0也将被传输
uint16 prn_code2              # PRN伪随机噪声码
float32 satellite_elevation2  # 卫星仰角，00~90度，前面的0也将被传输
float32 satellite_azimuth2    # 卫星方位角，000~359度，前面的0也将被传输
float32 signal_noise_ratio2   # 信噪比，00~99dB，没有跟踪到卫星时为空，前面的0也将被传输
uint16 prn_code3              # PRN伪随机噪声码
float32 satellite_elevation3  # 卫星仰角，00~90度，前面的0也将被传输
float32 satellite_azimuth3    # 卫星方位角，000~359度，前面的0也将被传输
float32 signal_noise_ratio3   # 信噪比，00~99dB，没有跟踪到卫星时为空，前面的0也将被传输
uint16 prn_code4              # PRN伪随机噪声码
float32 satellite_elevation4  # 卫星仰角，00~90度，前面的0也将被传输
float32 satellite_azimuth4    # 卫星方位角，000~359度，前面的0也将被传输
float32 signal_noise_ratio4   # 信噪比，00~99dB，没有跟踪到卫星时为空，前面的0也将被传输

