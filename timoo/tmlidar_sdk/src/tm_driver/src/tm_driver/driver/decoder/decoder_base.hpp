#pragma once
#include <tm_driver/common/common_header.h>
#include <tm_driver/utility/time.h>
#include <tm_driver/driver/driver_param.h>
namespace timoo
{
namespace lidar
{
#define DEFINE_MEMBER_CHECKER(member)                                                                                  \
  template <typename T, typename V = bool>                                                                             \
  struct has_##member : std::false_type                                                                                \
  {                                                                                                                    \
  };                                                                                                                   \
  template <typename T>                                                                                                \
  struct has_##member<                                                                                                 \
      T, typename std::enable_if<!std::is_same<decltype(std::declval<T>().member), void>::value, bool>::type>          \
    : std::true_type                                                                                                   \
  {                                                                                                                    \
  };
#define TM_HAS_MEMBER(C, member) has_##member<C>::value
DEFINE_MEMBER_CHECKER(x)
DEFINE_MEMBER_CHECKER(y)
DEFINE_MEMBER_CHECKER(z)
DEFINE_MEMBER_CHECKER(intensity)
DEFINE_MEMBER_CHECKER(ring)
DEFINE_MEMBER_CHECKER(timestamp)
#define TM_SWAP_SHORT(x) ((((x)&0xFF) << 8) | (((x)&0xFF00) >> 8))
#define TM_SWAP_LONG(x) ((((x)&0xFF) << 24) | (((x)&0xFF00) << 8) | (((x)&0xFF0000) >> 8) | (((x)&0xFF000000) >> 24))
#define TM_TO_RADS(x) ((x) * (M_PI) / 180)
constexpr float TM_DIS_RESOLUTION = 0.005;
constexpr float TM_HELIOS_DIS_RESOLUTION = 0.0025;
constexpr float TM_ANGLE_RESOLUTION = 0.01;
constexpr float MICRO = 1000000.0;
constexpr float NANO = 1000000000.0;
constexpr int TM_ONE_ROUND = 36000;
constexpr uint16_t PROTOCOL_VER_0 = 0x00;


/* Echo mode definition */
enum TMEchoMode
{
  ECHO_SINGLE,
  ECHO_DUAL
};

/* Decode result definition */
enum TMDecoderResult
{
  DECODE_OK = 0,
  FRAME_SPLIT = 1,
  WRONG_PKT_HEADER = -1,
  PKT_NULL = -2
};

#pragma pack(push, 1)
typedef struct
{
  uint64_t MSOP_ID;
  uint64_t DIFOP_ID;
  uint64_t BLOCK_ID;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;
  uint16_t TM64_FIRINGS_PER_BLOCK;
  uint16_t TM64_SCANS_PER_FIRING;
  uint16_t LASER_NUM;
  float PKT_RATE;
  float DSR_TOFFSET;
  float FIRING_TOFFSET;
  float RX;
  float RY;
  float RZ;
  uint8_t IMU_NUM_PER_GROUP;
  uint8_t IMU_ACC_RANGE;
  uint16_t IMU_GYRO_RANGE;
} LidarConstantParameter;

// typedef struct
// {
//   uint64_t MSOP_ID;
//   uint64_t DIFOP_ID;
//   uint64_t BLOCK_ID;
//   uint32_t PKT_RATE;
//   uint16_t BLOCKS_PER_PKT;
//   uint16_t TM64_FIRINGS_PER_BLOCK;
//   uint16_t TM64_SCANS_PER_FIRING;
//   uint16_t LASER_NUM;
  
//   float DSR_TOFFSET;
//   float FIRING_FREQUENCY;
//   float RX;
//   float RY;
//   float RZ;
// } TM64ConstantParameter;

typedef struct
{
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint16_t ms;
  uint16_t us;
} TMTimestampYMD;



typedef struct
{
  uint8_t sec[6];
  uint32_t us;
} TMTimestampUTC;

typedef struct
{
  uint16_t sync_state;
  TMTimestampYMD sync_time;
} TMTimeInfo;

typedef struct
{
  uint64_t id;
  uint8_t reserved_1[12];
  TMTimestampYMD timestamp;
  uint8_t lidar_type;
  uint8_t reserved_2[7];
  uint16_t temp_raw;
  uint8_t reserved_3[2];
} TMMsopHeader;

typedef struct
{
  uint32_t id;
  uint16_t protocol_version;
  uint8_t reserved_1;
  uint8_t wave_mode;
  uint8_t temp_low;
  uint8_t temp_high;
  TMTimestampUTC timestamp;
  uint8_t reserved_2[10];
  uint8_t lidar_type;
  uint8_t reserved_3[49];
} TMMsopHeaderNew;

typedef struct
{
  uint8_t lidar_ip[4];
  uint8_t host_ip[4];
  uint8_t mac_addr[6];
  uint16_t local_port;
  uint16_t dest_port;
  uint16_t port3;
  uint16_t port4;
} TMEthNet;

// ————————————————————————————————————————————————————————————————————————————————————————

// typedef struct
// {
//   uint8_t lidar_ip[4];
//   uint8_t dest_ip[4];
//   uint8_t mac_addr[6];
//   uint16_t msop_port;
//   uint16_t reserve_1;
//   uint16_t difop_port;
//   uint16_t reserve_2;
// } TMEthNetNew;

struct IP_NUMBER
{
	uint8_t Net_address_1;
	uint8_t Net_address_2;
	uint8_t Computer_address_1;
	uint8_t Computer_address_2;
};

struct ETH_REGISTER                                                          //以太网信息 26byte
{
	IP_NUMBER ip_src;
	IP_NUMBER ip_dest;
	uint8_t mac_addr[6];
	uint16_t port[6];
};

struct SERIAL_NUMBER
{
	uint8_t LINE_NUMBER;

	uint8_t OPERATING_RANGE;

	uint32_t NUMBER;
};

struct INTENSITYCALIBRATION{
	int8_t bvalue[64];
	uint8_t avalue[64];
	uint16_t seg[3];
	uint8_t Reserved[122];
};

struct LONGITUDEandLATITUDE                //longitude and latitude infomation 22bytes
{
	char LATITUDE[10];
	char LONGITUDE[10];
	char NorS;
	char WorE;
};

// ——————————————————————————————————————————————————————————————————————————————————————

typedef struct
{
  uint16_t start_angle;
  uint16_t end_angle;
} TMFOV;

typedef struct
{
  uint8_t sign;
  uint16_t value;
} TMCalibrationAngle;

typedef struct
{
  uint16_t distance;
  uint8_t intensity;
} TMChannel;

typedef struct
{
  uint8_t top_ver[5];
  uint8_t bottom_ver[5];
} TMVersion;

typedef struct
{
  uint8_t top_firmware_ver[5];
  uint8_t bot_firmware_ver[5];
  uint8_t bot_soft_ver[5];
  uint8_t motor_firmware_ver[5];
  uint8_t hw_ver[3];
} TMVersionNew;

typedef struct
{
  uint8_t num[6];
} TMSn;

typedef struct
{
  uint8_t device_current[3];
  uint8_t main_current[3];
  uint16_t vol_12v;
  uint16_t vol_sim_1v8;
  uint16_t vol_dig_3v3;
  uint16_t vol_sim_3v3;
  uint16_t vol_dig_5v4;
  uint16_t vol_sim_5v;
  uint16_t vol_ejc_5v;
  uint16_t vol_recv_5v;
  uint16_t vol_apd;
} TMStatus;

typedef struct
{
  uint16_t device_current;
  uint16_t vol_fpga;
  uint16_t vol_12v;
  uint16_t vol_dig_5v4;
  uint16_t vol_sim_5v;
  uint16_t vol_apd;
  uint8_t reserved[12];
} TMStatusNew;

typedef struct
{
  uint8_t reserved_1[9];
  uint16_t checksum;
  uint16_t manc_err1;
  uint16_t manc_err2;
  uint8_t gps_status;
  uint16_t temperature1;
  uint16_t temperature2;
  uint16_t temperature3;
  uint16_t temperature4;
  uint16_t temperature5;
  uint8_t reserved_2[5];
  uint16_t cur_rpm;
  uint8_t reserved_3[7];
} TMDiagno;

typedef struct
{
  uint16_t bot_fpga_temperature;
  uint16_t recv_A_temperature;
  uint16_t recv_B_temperature;
  uint16_t main_fpga_temperature;
  uint16_t main_fpga_core_temperature;
  uint16_t real_rpm;
  uint8_t lane_up;
  uint16_t lane_up_cnt;
  uint16_t main_status;
  uint8_t gps_status;
  uint8_t reserved[22];
} TMDiagnoNew;

typedef struct{
  int16_t IMU_Acceleration_x;
  int16_t IMU_Acceleration_y;
  int16_t IMU_Acceleration_z;
  int16_t IMU_Angular_velocity_x;
  int16_t IMU_Angular_velocity_y;
  int16_t IMU_Angular_velocity_z;
}TM_IMUDATA;

typedef struct{
  uint16_t IMU_TIME[3];
  int16_t IMU_TEMP;
  TM_IMUDATA IMU_DATA[4];
}TM_IMU;


#pragma pack(pop)

//----------------- Decoder ---------------------
template <typename T_Point>
class DecoderBase
{
public:
  explicit DecoderBase(const TMDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  DecoderBase(const DecoderBase&) = delete;
  DecoderBase& operator=(const DecoderBase&) = delete;
  virtual ~DecoderBase() = default;
  virtual TMDecoderResult processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& point_cloud_vec, int& height);
  virtual TMDecoderResult processDifopPkt(const uint8_t* pkt);
  virtual TMDecoderResult processDifopPkt(const uint8_t* pkt, std::vector<TmImu>& imu_vec);
  virtual void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback);  ///< Camera trigger
  virtual double getLidarTemperature();
  virtual double getLidarTime(const uint8_t* pkt) = 0;
  virtual double getIMUTime(const uint8_t* pkt) = 0;

protected:
  virtual float computeTemperature(const uint16_t& temp_raw);
  virtual float computeTemperature(const uint8_t& temp_low, const uint8_t& temp_high);
  virtual int azimuthCalibration(const float& azimuth, const int& channel);
  virtual void checkTriggerAngle(const int& angle, const double& timestamp);
  virtual TMDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth) = 0;
  virtual TMDecoderResult decodeDifopPkt(const uint8_t* pkt) = 0;
  virtual TMDecoderResult decodeDifopPkt(const uint8_t* pkt,std::vector<TmImu>& imu_vec) = 0;
  TMEchoMode getEchoMode(const LidarType& type, const uint8_t& return_mode);
  template <typename T_Msop>
  double calculateTimeUTC(const uint8_t* pkt, const LidarType& type);
  template <typename T_Msop>
  double calculateTimeYMD(const uint8_t* pkt);
  template <typename T_Difop>
  double calculateIMUTimeYMD(const uint8_t* pkt);
  template <typename T_Difop>
  void decodeDifopCommon(const uint8_t* pkt, const LidarType& type);
  template <typename T_Difop>
  void decodeDifopCalibration(const uint8_t* pkt, const LidarType& type);
  void transformPoint(float& x, float& y, float& z);
  float checkCosTable(const int& angle);
  float checkSinTable(const int& angle);
  void sortBeamTable();

private:
  std::vector<double> initTrigonometricLookupTable(const std::function<double(const double)>& func);
  
protected:
  const LidarConstantParameter lidar_const_param_;
  TMDecoderParam param_;
  TMEchoMode echo_mode_;
  unsigned int pkts_per_frame_;
  unsigned int pkt_count_;
  unsigned int trigger_index_;
  unsigned int prev_angle_diff_;
  unsigned int rpm_;
  unsigned int protocol_ver_;
  int start_angle_;
  int end_angle_;
  int cut_angle_;
  int last_azimuth_;
  bool angle_flag_;
  bool difop_flag_;
  float fov_time_jump_diff_;
  float time_duration_between_blocks_;
  float current_temperature_;
  float azi_diff_between_block_theoretical_;
  float time_duration_between_firing_;

  TMTimeInfo gps_time_;   //gps time
  std::vector<int> vert_angle_list_;
  std::vector<int> hori_angle_list_;
  std::vector<uint16_t> beam_ring_table_;
  std::vector<std::function<void(const CameraTrigger&)>> camera_trigger_cb_vec_;
  std::function<double(const uint8_t*)> get_point_time_func_;
  std::function<double(const uint8_t*)> get_imu_time_func_;
  std::function<void(const int&, const uint8_t*)> check_camera_trigger_func_;
private:
  std::vector<double> cos_lookup_table_;
  std::vector<double> sin_lookup_table_;

};

template <typename T_Point>
inline DecoderBase<T_Point>::DecoderBase(const TMDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : lidar_const_param_(lidar_const_param)
  , param_(param)
  , echo_mode_(ECHO_SINGLE)
  , pkts_per_frame_(ceil(lidar_const_param.PKT_RATE / 10))
  , pkt_count_(0)
  , trigger_index_(0)
  , prev_angle_diff_(TM_ONE_ROUND)
  , rpm_(600)
  , protocol_ver_(0)
  , start_angle_(param.start_angle * 100)
  , end_angle_(param.end_angle * 100)
  , cut_angle_(param.cut_angle * 100)
  , last_azimuth_(-36001)
  , angle_flag_(true)
  , difop_flag_(false)
  , fov_time_jump_diff_(0)
  , time_duration_between_blocks_(0)
  , current_temperature_(0)
  , azi_diff_between_block_theoretical_(20)
  , time_duration_between_firing_(3.072)
{
  if (cut_angle_ > TM_ONE_ROUND)
  {
    cut_angle_ = 0;
  }
  if (this->start_angle_ > TM_ONE_ROUND || this->start_angle_ < 0 || this->end_angle_ > TM_ONE_ROUND ||
      this->end_angle_ < 0)
  {
    TM_WARNING << "start_angle & end_angle should be in range 0~360° " << TM_REND;
    this->start_angle_ = 0;
    this->end_angle_ = TM_ONE_ROUND;
  }
  if (this->start_angle_ > this->end_angle_)
  {
    this->angle_flag_ = false;
  }

  /* Point time function*/
  if (TM_HAS_MEMBER(T_Point, timestamp))  ///< return the timestamp of the first block in one packet
  {
    if (this->param_.use_lidar_clock)
    {
      get_point_time_func_ = [this](const uint8_t* pkt) { return getLidarTime(pkt)- (this->lidar_const_param_.BLOCKS_PER_PKT *  this->lidar_const_param_.CHANNELS_PER_BLOCK - 1) * this->lidar_const_param_.DSR_TOFFSET * 1e-6; };
    }
    else
    {
      get_point_time_func_ = [this](const uint8_t* pkt) {
        double ret_time = getTime() - (this->lidar_const_param_.BLOCKS_PER_PKT *  this->lidar_const_param_.CHANNELS_PER_BLOCK - 1) * this->lidar_const_param_.DSR_TOFFSET * 1e-6 ;
        return ret_time;
      };
    }
  }
  else
  {
    get_point_time_func_ = [this](const uint8_t* pkt) { return 0; };
  }
  /*IMU time function*/
   
  if (this->param_.use_lidar_clock)
  {
    get_imu_time_func_ = [this](const uint8_t* pkt) { return getIMUTime(pkt); };
  }
  else
  {
    get_imu_time_func_ = [this](const uint8_t* pkt) {
      double ret_time = getTime();
      return ret_time;
      
    };
  }


  /*Camera trigger function*/
  if (param.trigger_param.trigger_map.size() != 0)
  {
    if (this->param_.use_lidar_clock)
    {
      check_camera_trigger_func_ = [this](const int& azimuth, const uint8_t* pkt) {
        checkTriggerAngle(azimuth, getLidarTime(pkt));
      };
    }
    else
    {
      check_camera_trigger_func_ = [this](const int& azimuth, const uint8_t* pkt) {
        checkTriggerAngle(azimuth, getTime());
      };
    }
  }
  else
  {
    check_camera_trigger_func_ = [this](const int& azimuth, const uint8_t* pkt) { return; };
  }

  /* Cos & Sin look-up table*/
  cos_lookup_table_ = initTrigonometricLookupTable([](const double rad) -> double { return std::cos(rad); });
  sin_lookup_table_ = initTrigonometricLookupTable([](const double rad) -> double { return std::sin(rad); });
}

template <typename T_Point>
inline TMDecoderResult DecoderBase<T_Point>::processDifopPkt(const uint8_t* pkt)
{
  if (pkt == NULL)
  {
    return PKT_NULL;
  }
  return decodeDifopPkt(pkt);
}

template <typename T_Point>
inline TMDecoderResult DecoderBase<T_Point>::processDifopPkt(const uint8_t* pkt, std::vector<TmImu>& imu_vec)
{
  if (pkt == NULL)
  {
    return PKT_NULL;
  }
  return decodeDifopPkt(pkt,imu_vec);
}

template <typename T_Point>
inline TMDecoderResult DecoderBase<T_Point>::processMsopPkt(const uint8_t* pkt, std::vector<T_Point>& point_cloud_vec,
                                                            int& height)
{
  if (pkt == NULL)
  {
    return PKT_NULL;
  }

  int azimuth = 0;
  TMDecoderResult ret = decodeMsopPkt(pkt, point_cloud_vec, height, azimuth);
  if (ret != TMDecoderResult::DECODE_OK)
  {
    return ret;
  }
  this->pkt_count_++;
  switch (this->param_.split_frame_mode)
  {
    case SplitFrameMode::SPLIT_BY_ANGLE:
      if (azimuth < this->last_azimuth_)
      {
        this->last_azimuth_ -= TM_ONE_ROUND;
      }
      if (this->last_azimuth_ != -36001 && this->last_azimuth_ < this->cut_angle_ && azimuth >= this->cut_angle_)
      {
        this->last_azimuth_ = azimuth;
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = TM_ONE_ROUND;
        return FRAME_SPLIT;
      }
      this->last_azimuth_ = azimuth;
      break;
    case SplitFrameMode::SPLIT_BY_FIXED_PKTS:
      if (this->pkt_count_ >= this->pkts_per_frame_)
      {
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = TM_ONE_ROUND;
        return FRAME_SPLIT;
      }
      break;
    case SplitFrameMode::SPLIT_BY_CUSTOM_PKTS:
      if (this->pkt_count_ >= this->param_.num_pkts_split)
      {
        this->pkt_count_ = 0;
        this->trigger_index_ = 0;
        this->prev_angle_diff_ = TM_ONE_ROUND;
        return FRAME_SPLIT;
      }
      break;
    default:
      break;
  }
  return DECODE_OK;
}

template <typename T_Point>
inline void DecoderBase<T_Point>::regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
{
  camera_trigger_cb_vec_.emplace_back(callback);
}

template <typename T_Point>
inline double DecoderBase<T_Point>::getLidarTemperature()
{
  return current_temperature_;
}

template <typename T_Point>
inline void DecoderBase<T_Point>::checkTriggerAngle(const int& angle, const double& timestamp)
{
  std::map<double, std::string>::iterator iter = param_.trigger_param.trigger_map.begin();
  for (size_t i = 0; i < trigger_index_; i++)
  {
    iter++;
  }
  if (iter != param_.trigger_param.trigger_map.end())
  {
    unsigned int angle_diff = std::abs(iter->first * 100 - angle);
    if (angle_diff < prev_angle_diff_)
    {
      prev_angle_diff_ = angle_diff;
      return;
    }
    else
    {
      trigger_index_++;
      prev_angle_diff_ = TM_ONE_ROUND;
      for (auto cb : camera_trigger_cb_vec_)
      {
        cb(std::make_pair(iter->second, timestamp));
      }
    }
  }
}

/* 16, 32, BP & RSHELIOS */
template <typename T_Point>
inline float DecoderBase<T_Point>::computeTemperature(const uint16_t& temp_raw)
{
  uint8_t neg_flag = (temp_raw >> 8) & 0x80;
  float msb = (temp_raw >> 8) & 0x7F;
  float lsb = (temp_raw & 0x00FF) >> 3;
  float temp;
  if (neg_flag == 0x80)
  {
    temp = -1 * (msb * 32 + lsb) * 0.0625f;
  }
  else
  {
    temp = (msb * 32 + lsb) * 0.0625f;
  }

  return temp;
}

/* 128 & 80 */
template <typename T_Point>
inline float DecoderBase<T_Point>::computeTemperature(const uint8_t& temp_low, const uint8_t& temp_high)
{
  uint8_t neg_flag = temp_low & 0x80;
  float msb = temp_low & 0x7F;
  float lsb = temp_high >> 4;
  float temp;
  if (neg_flag == 0x80)
  {
    temp = -1 * (msb * 16 + lsb) * 0.0625f;
  }
  else
  {
    temp = (msb * 16 + lsb) * 0.0625f;
  }

  return temp;
}

template <typename T_Point>
inline int DecoderBase<T_Point>::azimuthCalibration(const float& azimuth, const int& channel)
{
  return (static_cast<int>(azimuth) + this->hori_angle_list_[channel] + TM_ONE_ROUND) % TM_ONE_ROUND;
}

template <typename T_Point>
template <typename T_Difop>
inline void DecoderBase<T_Point>::decodeDifopCommon(const uint8_t* pkt, const LidarType& type)
{
  const T_Difop* dpkt_ptr = reinterpret_cast<const T_Difop*>(pkt);
  // this->echo_mode_ = this->getEchoMode(type, dpkt_ptr->return_mode);
  this->echo_mode_ = TMEchoMode::ECHO_SINGLE;
  this->rpm_ = dpkt_ptr->rpm;
  if (this->rpm_ == 0)
  {
    TM_WARNING << "LiDAR RPM is 0" << TM_REND;
    this->rpm_ = 600;
  }
  this->time_duration_between_blocks_ =
      (60 / static_cast<float>(this->rpm_)) /
      ((this->lidar_const_param_.PKT_RATE * 60 / this->rpm_) * this->lidar_const_param_.BLOCKS_PER_PKT);
  // int fov_start_angle = TM_SWAP_SHORT(dpkt_ptr->fov.start_angle);
  // int fov_end_angle = TM_SWAP_SHORT(dpkt_ptr->fov.end_angle);
  int fov_start_angle = 0;
  int fov_end_angle = 36000;
  int fov_range = (fov_start_angle < fov_end_angle) ? (fov_end_angle - fov_start_angle) :
                                                      (TM_ONE_ROUND - fov_start_angle + fov_end_angle);
  float blocks_per_round =
      (this->lidar_const_param_.PKT_RATE / (this->rpm_ / 60)) * this->lidar_const_param_.BLOCKS_PER_PKT;
  this->fov_time_jump_diff_ =
      this->time_duration_between_blocks_ * (fov_range / (TM_ONE_ROUND / static_cast<float>(blocks_per_round)));
  if (this->echo_mode_ == TMEchoMode::ECHO_DUAL)
  {
    this->pkts_per_frame_ = ceil(2 * this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }
  else
  {
    this->pkts_per_frame_ = ceil(this->lidar_const_param_.PKT_RATE * 60 / this->rpm_);
  }
  this->azi_diff_between_block_theoretical_ =
      (TM_ONE_ROUND / this->lidar_const_param_.BLOCKS_PER_PKT) /
      static_cast<float>(this->pkts_per_frame_);  ///< ((rpm/60)*360)/pkts_rate/blocks_per_pkt
}

template <typename T_Point>
template <typename T_Difop>
inline void DecoderBase<T_Point>::decodeDifopCalibration(const uint8_t* pkt, const LidarType& type)
{
  const T_Difop* dpkt_ptr = reinterpret_cast<const T_Difop*>(pkt);
  const uint8_t* p_ver_cali = reinterpret_cast<const uint8_t*>(dpkt_ptr->ver_angle_cali);
  if ((p_ver_cali[0] == 0x00 || p_ver_cali[0] == 0xFF) && (p_ver_cali[1] == 0x00 || p_ver_cali[1] == 0xFF) &&
      (p_ver_cali[2] == 0x00 || p_ver_cali[2] == 0xFF) && (p_ver_cali[3] == 0x00 || p_ver_cali[3] == 0xFF))
  {
    return;
  }
  int neg = 1;
  for (size_t i = 0; i < this->lidar_const_param_.LASER_NUM; i++)
  {
    /* vert angle calibration data */
    neg = static_cast<int>(dpkt_ptr->ver_angle_cali[i].sign) == 0 ? 1 : -1;
    this->vert_angle_list_[i] = static_cast<int>(TM_SWAP_SHORT(dpkt_ptr->ver_angle_cali[i].value)) * neg;
    /* horizon angle calibration data */
    neg = static_cast<int>(dpkt_ptr->hori_angle_cali[i].sign) == 0 ? 1 : -1;
    this->hori_angle_list_[i] = static_cast<int>(TM_SWAP_SHORT(dpkt_ptr->hori_angle_cali[i].value)) * neg;
    if (type == LidarType::TM32)
    {
      this->vert_angle_list_[i] *= 0.1f;
      this->hori_angle_list_[i] *= 0.1f;
    }
  }
  this->sortBeamTable();
  this->difop_flag_ = true;
}

template <typename T_Point>
template <typename T_Msop>
inline double DecoderBase<T_Point>::calculateTimeUTC(const uint8_t* pkt, const LidarType& type)
{
  // const T_Msop* mpkt_ptr = reinterpret_cast<const T_Msop*>(pkt);
  // union u_ts
  // {
  //   uint8_t data[8];
  //   uint64_t ts;
  // } timestamp;
  // timestamp.data[7] = 0;
  // timestamp.data[6] = 0;
  // timestamp.data[5] = mpkt_ptr->header.timestamp.sec[0];
  // timestamp.data[4] = mpkt_ptr->header.timestamp.sec[1];
  // timestamp.data[3] = mpkt_ptr->header.timestamp.sec[2];
  // timestamp.data[2] = mpkt_ptr->header.timestamp.sec[3];
  // timestamp.data[1] = mpkt_ptr->header.timestamp.sec[4];
  // timestamp.data[0] = mpkt_ptr->header.timestamp.sec[5];

  // if ((type == LidarType::TM80 || type == LidarType::TM128) && this->protocol_ver_ == PROTOCOL_VER_0)
  // {
  //   return static_cast<double>(timestamp.ts) +
  //          (static_cast<double>(TM_SWAP_LONG(mpkt_ptr->header.timestamp.us))) / NANO;
  // }

  // return static_cast<double>(timestamp.ts) + (static_cast<double>(TM_SWAP_LONG(mpkt_ptr->header.timestamp.us))) / MICRO;
  return 1;
}

template <typename T_Point>
template <typename T_Msop>
inline double DecoderBase<T_Point>::calculateTimeYMD(const uint8_t* pkt)
{
  #ifdef _MSC_VER
  long timezone = 0;
  _get_timezone(&timezone);
  #endif
  const T_Msop* mpkt_ptr = reinterpret_cast<const T_Msop*>(pkt);
  std::tm stm;
  memset(&stm, 0, sizeof(stm));
  if(gps_time_.sync_state == 0x55aa){
    stm.tm_year = gps_time_.sync_time.year + 100;
    stm.tm_mon = gps_time_.sync_time.month - 1;
    stm.tm_mday = gps_time_.sync_time.day;
    stm.tm_hour = gps_time_.sync_time.hour ;
  }else{
    stm.tm_year = 70;
    stm.tm_mon = 0;
    stm.tm_mday = 1;
    stm.tm_hour = 0;
  }
  
  //test time code
  // std::tm *local_timeinfo;
  // time_t local_time;
  // time(&local_time);
  // local_timeinfo = localtime(&local_time);
  // printf("now_timestamp: %ld \n ", std::mktime(local_timeinfo));
  // printf("gps_timestamp: %lf \n ", std::mktime(&stm) + static_cast<double>(mpkt_ptr->timestamp) / 1000000.0 - static_cast<double>(timezone));
  return std::mktime(&stm) + static_cast<double>(mpkt_ptr->timestamp) / 1000000.0 - static_cast<double>(timezone);
}


template <typename T_Point>
template <typename T_Difop>
inline double DecoderBase<T_Point>::calculateIMUTimeYMD(const uint8_t* pkt)
{
  #ifdef _MSC_VER
  long timezone = 0;
  _get_timezone(&timezone);
  #endif
  const T_Difop* dpkt_ptr = reinterpret_cast<const T_Difop*>(pkt);
  std::tm stm;
  memset(&stm, 0, sizeof(stm));

  if(gps_time_.sync_state == 0x55aa){
    stm.tm_year = gps_time_.sync_time.year + 100;
    stm.tm_mon = gps_time_.sync_time.month - 1;
    stm.tm_mday = gps_time_.sync_time.day;
    stm.tm_hour = gps_time_.sync_time.hour;
  }else{
    stm.tm_year = 70;
    stm.tm_mon = 0;
    stm.tm_mday = 1;
    stm.tm_hour = 0;
  }
  stm.tm_min = 0;
  stm.tm_sec = 0;
  
  // printf("imu time = %lf\n",std::mktime(&stm) + static_cast<double>(ntohl(dpkt_ptr->IMU_Time_Start)) / 1000000.0 - static_cast<double>(timezone));
  return std::mktime(&stm) + static_cast<double>(ntohl(dpkt_ptr->IMU_Time_Start)) / 1000000.0 - static_cast<double>(timezone);
}


template <typename T_Point>
inline void DecoderBase<T_Point>::transformPoint(float& x, float& y, float& z)
{
#ifdef ENABLE_TRANSFORM
  Eigen::AngleAxisd current_rotation_x(param_.transform_param.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(param_.transform_param.pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(param_.transform_param.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y,
                                           param_.transform_param.z);
  Eigen::Matrix4d trans = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
  Eigen::Vector4d target_ori(x, y, z, 1);
  Eigen::Vector4d target_rotate = trans * target_ori;
  x = target_rotate(0);
  y = target_rotate(1);
  z = target_rotate(2);
#endif
}

template <typename T_Point>
inline void DecoderBase<T_Point>::sortBeamTable()
{
  std::vector<size_t> sorted_idx(this->lidar_const_param_.LASER_NUM);
  std::iota(sorted_idx.begin(), sorted_idx.end(), 0);
  std::sort(sorted_idx.begin(), sorted_idx.end(), [this](std::size_t i1, std::size_t i2) -> bool {
    return this->vert_angle_list_[i1] < this->vert_angle_list_[i2];
  });
  for (size_t i = 0; i < this->lidar_const_param_.LASER_NUM; i++)
  {
    this->beam_ring_table_[sorted_idx[i]] = i;
  }
}

template <typename T_Point>
inline typename std::enable_if<!TM_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<TM_HAS_MEMBER(T_Point, x)>::type setX(T_Point& point, const float& value)
{
  point.x = value;
}

template <typename T_Point>
inline typename std::enable_if<!TM_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<TM_HAS_MEMBER(T_Point, y)>::type setY(T_Point& point, const float& value)
{
  point.y = value;
}

template <typename T_Point>
inline typename std::enable_if<!TM_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
}

template <typename T_Point>
inline typename std::enable_if<TM_HAS_MEMBER(T_Point, z)>::type setZ(T_Point& point, const float& value)
{
  point.z = value;
}

template <typename T_Point>
inline typename std::enable_if<!TM_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                      const uint8_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<TM_HAS_MEMBER(T_Point, intensity)>::type setIntensity(T_Point& point,
                                                                                     const uint8_t& value)
{
  point.intensity = value;
}

template <typename T_Point>
inline typename std::enable_if<!TM_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint16_t& value)
{
}

template <typename T_Point>
inline typename std::enable_if<TM_HAS_MEMBER(T_Point, ring)>::type setRing(T_Point& point, const uint16_t& value)
{
  point.ring = value;
}

template <typename T_Point>
inline typename std::enable_if<!TM_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                      const double& value)
{
}

template <typename T_Point>
inline typename std::enable_if<TM_HAS_MEMBER(T_Point, timestamp)>::type setTimestamp(T_Point& point,
                                                                                     const double& value)
{
  point.timestamp = value;
}

template <typename T_Point>
inline TMEchoMode DecoderBase<T_Point>::getEchoMode(const LidarType& type, const uint8_t& return_mode)
{
  switch (type)
  {
    case LidarType::TM16:
    case LidarType::TM32:
    case LidarType::TM32:
    case LidarType::TM64:
    case LidarType::TM128:
    case LidarType::TM300:
      switch (return_mode)
      {
        case 0x00:
          return TMEchoMode::ECHO_DUAL;
        default:
          return TMEchoMode::ECHO_SINGLE;
      }
  }
  return TMEchoMode::ECHO_SINGLE;
}

template <typename T_Point>
inline float DecoderBase<T_Point>::checkCosTable(const int& angle)
{
  return cos_lookup_table_[angle + TM_ONE_ROUND];
}
template <typename T_Point>
inline float DecoderBase<T_Point>::checkSinTable(const int& angle)
{
  return sin_lookup_table_[angle + TM_ONE_ROUND];
}

template <typename T_Point>
inline std::vector<double>
DecoderBase<T_Point>::initTrigonometricLookupTable(const std::function<double(const double)>& func)
{
  std::vector<double> temp_table = std::vector<double>(2 * TM_ONE_ROUND, 0.0);
  for (int i = 0; i < 2 * TM_ONE_ROUND; i++)
  {
    const double rad = TM_TO_RADS(static_cast<double>(i - TM_ONE_ROUND) * TM_ANGLE_RESOLUTION);
    temp_table[i] = func(rad);
  }
  return temp_table;
}

}  // namespace lidar
}  // namespace timoo