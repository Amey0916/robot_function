#include <tm_driver/driver/decoder/decoder_base.hpp>
namespace timoo
{
namespace lidar
{
#pragma pack(push, 1)
typedef struct
{
  uint16_t id;
  uint16_t azimuth;
  TMChannel channels[32];
} TM64MsopBlock;

typedef struct
{
  TM64MsopBlock blocks[12];
  uint32_t timestamp;
  uint16_t tail;
} TM64MsopPkt;

union Float4Bytes{
  uint8_t bytes[4];
  float value;
};

typedef struct
{
  uint64_t id;                                              //DIFOP识别头             8byte
  uint16_t rpm;                                               //雷达转速                2byte
  ETH_REGISTER Ethernet_info;                                 //以太网信息              26byte                                                                   4byte 
  TMTimestampYMD gps_time;                                   //雷达时间               10byte

  uint8_t IsRotationorStatic;                                 //雷达旋转/静止           1byte
  uint8_t WorkModel;									                      	//雷达工作模式            1byte

  SERIAL_NUMBER Serial_number;                                //序列号                 6byte
  uint8_t Top_firmware_version[5];			                  		//顶板固件版本号          5byte
  uint8_t Bottom_firmware_version[5];			                  	//底板固件版本号          5byte
  uint16_t Phrase_loc;                                        //电机锁相相位            2byte

  uint8_t Distance_calibration[512];			                   	//距离校准（预留）        512byte
  INTENSITYCALIBRATION Intensity_calibration;									//强度校准（预留）        256byte
  int8_t VerticalAngle_calibration[128];									    //垂直角度校准(预留）     128byte
  int16_t Azimuth_calibration;						                		//水平角度校准（预留）    2byte

  LONGITUDEandLATITUDE GPS_Position;                          //经纬度                 22byte
  uint16_t BlockCoef[7];                                      //                       14byte
  uint16_t GPS_state;	                                      //GPS状态信息             2byte
  uint16_t DenoiseSeg[5];                                     //                       10byte
  uint16_t Reserved1;                                         //                       2byte
  int8_t BoardAzimuth;                                        //                       1byte
  int8_t Plane4Azimuth[4];                                    //                       4byte

  //IMU
  uint32_t IMU_Time_Start;
  uint32_t IMU_Time_End;
  uint16_t IMU_Temp;
  IMU_DATA IMU_data[4];
  

  uint8_t Reserved_2[127];
  uint8_t Tail[2];
}TM64DifopPkt;

#pragma pack(pop)

template <typename T_Point>
class DecoderTM64 : public DecoderBase<T_Point>
{
public:
  explicit DecoderTM64(const TMDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  TMDecoderResult decodeDifopPkt(const uint8_t* pkt);
  TMDecoderResult decodeDifopPkt(const uint8_t* pkt, std::vector<TmImu>& imu_vec);
  TMDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  double getLidarTime(const uint8_t* pkt);
  double getIMUTime(const uint8_t* pkt);

  double cur_timestamp;
  double last_timestamp = 0;
};

template <typename T_Point>
inline DecoderTM64<T_Point>::DecoderTM64(const TMDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_Point>(param, lidar_const_param)
{
  this->vert_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->hori_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->beam_ring_table_.resize(this->lidar_const_param_.LASER_NUM);
  if (this->param_.max_distance > 200.0f)
  {
    this->param_.max_distance = 200.0f;
  }
  if (this->param_.min_distance < 0.2f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.2f;
  }
}

template <typename T_Point>
inline double DecoderTM64<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeYMD<TM64MsopPkt>(pkt);
}

template <typename T_Point>
inline double DecoderTM64<T_Point>::getIMUTime(const uint8_t* pkt)
{
  return this->template calculateIMUTimeYMD<TM64DifopPkt>(pkt);
}

template <typename T_Point>
inline TMDecoderResult DecoderTM64<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                           int& azimuth)
{   
    height = this->lidar_const_param_.LASER_NUM;
    const TM64MsopPkt* mpkt_ptr = reinterpret_cast<const TM64MsopPkt*>(pkt);
    //Add judgment on packet headers
    // if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
    // {
    //   return TMDecoderResult::WRONG_PKT_HEADER;
    // }
    azimuth = (mpkt_ptr->blocks[0].azimuth);
    
    double block_timestamp = this->get_point_time_func_(pkt); //first block timestamp && first point timestamp per pkt

    float azi_diff = 0;
    for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
    {
      if (mpkt_ptr->blocks[blk_idx].id != this->lidar_const_param_.BLOCK_ID)
      {
        break;
      }
      int cur_azi  = (mpkt_ptr->blocks[blk_idx].azimuth);
      if (this->echo_mode_ == ECHO_DUAL){
            if(blk_idx < 10 ){
              azi_diff = static_cast<float>((TM_ONE_ROUND + (mpkt_ptr->blocks[blk_idx + 2].azimuth) - cur_azi) %
                                      TM_ONE_ROUND);
            }
            if(blk_idx > 0 && blk_idx % 2 == 0){
                  block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                                        (block_timestamp + this->time_duration_between_blocks_);
            }

      } 
      else{
        if(blk_idx < 11 ){
              azi_diff = static_cast<float>((TM_ONE_ROUND + (mpkt_ptr->blocks[blk_idx + 1].azimuth) - cur_azi) %
                                      TM_ONE_ROUND);
          }
          if(blk_idx > 0){
            block_timestamp = (azi_diff > 100) ? (block_timestamp + this->fov_time_jump_diff_) :
                                                  (block_timestamp + this->time_duration_between_blocks_);
          }
      }     
      azi_diff = (azi_diff > 100) ? this->azi_diff_between_block_theoretical_ : azi_diff;
      for (size_t channel_idx = 0; channel_idx < this->lidar_const_param_.CHANNELS_PER_BLOCK; channel_idx++)
      {
        float azi_channel_ori =
              cur_azi + (azi_diff * ((channel_idx % 16 * this->lidar_const_param_.DSR_TOFFSET) + 
              ( channel_idx / 16 * this->lidar_const_param_.FIRING_TOFFSET)) / (2* this->lidar_const_param_.FIRING_TOFFSET ));
      
        int mirror_index = -1;
        int two_plank_azi_diff = 0;
        if(channel_idx % 2 != 0)
          two_plank_azi_diff = 216;
        int map_mirror_idx_to_index[4] = {0,2,1,3}; //jin mian yin she
        for(size_t mirror_idx = 0; mirror_idx < 4; mirror_idx++){
          if(azi_channel_ori>1500+mirror_idx*9000 && azi_channel_ori < 7500+mirror_idx*9000){
            mirror_index = map_mirror_idx_to_index[mirror_idx];
              azi_channel_ori = (azi_channel_ori - 1500+mirror_idx*9000 - two_plank_azi_diff) * 2 - 6000;
            break;
         }
        }

        int azi_channel_final = azi_channel_ori;
        float distance = (mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * TM_DIS_RESOLUTION;
        int angle_horiz_ori = static_cast<int>(azi_channel_ori + TM_ONE_ROUND) % TM_ONE_ROUND;
        int angle_vert = ((this->vert_angle_list_[mirror_index * 16 + channel_idx % 16]) + TM_ONE_ROUND) % TM_ONE_ROUND;
        T_Point point;
        if (mirror_index != -1 && (distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
            ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
            (!this->angle_flag_ && ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))
        {
          float x = distance * this->checkCosTable(angle_vert) * this->checkCosTable(azi_channel_final) +
                    this->lidar_const_param_.RX * this->checkCosTable(angle_horiz_ori);
          float y = -distance * this->checkCosTable(angle_vert) * this->checkSinTable(azi_channel_final) -
                    this->lidar_const_param_.RX * this->checkSinTable(angle_horiz_ori);
          float z = distance * this->checkSinTable(angle_vert) + this->lidar_const_param_.RZ;
          uint8_t intensity = mpkt_ptr->blocks[blk_idx].channels[channel_idx].intensity;
          this->transformPoint(x, y, z);
          setX(point, x);
          setY(point, y);
          setZ(point, z);
          setIntensity(point, intensity);
        }
        else
        {
          setX(point, NAN);
          setY(point, NAN);
          setZ(point, NAN);
          setIntensity(point, 0);
        }
        setRing(point, this->beam_ring_table_[mirror_index * 16 + channel_idx % 16]);

        double timestamp = block_timestamp + channel_idx * this->time_duration_between_firing_ * 1e-6;
        
        setTimestamp(point, timestamp);
        vec.emplace_back(std::move(point));
    }
  }
  return TMDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline TMDecoderResult DecoderTM64<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  const TM64DifopPkt* dpkt_ptr = reinterpret_cast<const TM64DifopPkt*>(pkt);
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return TMDecoderResult::WRONG_PKT_HEADER;
  }
  this->template decodeDifopCommon<TM64DifopPkt>(pkt, LidarType::TM64);
  if (!this->difop_flag_)
  {
    float VerticalDefaultTM64[16] = {10.690 , 9.333 , 7.977 , 6.620 , 5.264 , 3.908 , 2.552 ,
                               1.196 ,-0.160 , -1.515 , -2.871 , -4.227 , -5.583 , -6.939 , -8.296 , -9.653};

    float VERT_ANGLE_64[4][16];
    
    float verticalTM64[16];

    for(int i = 0;i<16;i++){
      Float4Bytes float4bytes;
      for(int j = 0;j<4;j++){
        float4bytes.bytes[j] = dpkt_ptr->VerticalAngle_calibration[i*4 + j];
      }
      if(float4bytes.value >= VerticalDefaultTM64[i] - 1 && float4bytes.value <= VerticalDefaultTM64[i] + 1){
        verticalTM64[i] = float4bytes.value;
      }else{
        verticalTM64[i] = VerticalDefaultTM64[i];
      }
    }

    float diff = 0;
    for(int i = 0 ; i<16;i++){
      if(i<15){
        diff = (verticalTM64[i] - verticalTM64[i + 1]) / 4; 
        VERT_ANGLE_64[0][i] = verticalTM64[i];
        for(int j = 1; j < 4; j++){
          VERT_ANGLE_64[j][i] = verticalTM64[i] - j*diff;
        }
      }else{
        VERT_ANGLE_64[0][15] = verticalTM64[15];
        for(int j = 1; j < 4; j++){
          VERT_ANGLE_64[j][15] = verticalTM64[15] - j*diff; 
        }
      }
      
    }

    for(size_t i = 0;i<4;i++){
      for(size_t j = 0; j<16 ; j++){
          this->vert_angle_list_[i*16+j] =  static_cast<int>(VERT_ANGLE_64[i][j]*100);
      }
    }
    this->sortBeamTable();
    this->difop_flag_ = true;
  }
   //Constantly refreshing time
  this->gps_time_.sync_state = dpkt_ptr->GPS_state;
  this->gps_time_.sync_time.year = dpkt_ptr->gps_time.year;
  this->gps_time_.sync_time.month = dpkt_ptr->gps_time.month;
  this->gps_time_.sync_time.day = dpkt_ptr->gps_time.day;
  this->gps_time_.sync_time.hour = dpkt_ptr->gps_time.hour;
  this->gps_time_.sync_time.minute = 0;
  this->gps_time_.sync_time.second = 0;
  this->gps_time_.sync_time.ms = 0;
  this->gps_time_.sync_time.us = 0;
  return TMDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline TMDecoderResult DecoderTM64<T_Point>::decodeDifopPkt(const uint8_t* pkt, std::vector<TmImu>& imu_vec){
  const TM64DifopPkt* dpkt_ptr = reinterpret_cast<const TM64DifopPkt*>(pkt);
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return TMDecoderResult::WRONG_PKT_HEADER;
  }
  this->template decodeDifopCommon<TM64DifopPkt>(pkt, LidarType::TM64);
  if (!this->difop_flag_)
  {
   float VerticalDefaultTM64[16] = {10.690 , 9.333 , 7.977 , 6.620 , 5.264 , 3.908 , 2.552 , 
                            1.196 ,-0.160 , -1.515 , -2.871 , -4.227 , -5.583 , -6.939 , -8.296 , -9.653};

    float VERT_ANGLE_64[4][16];

    float verticalTM64[16];

    for(int i = 0;i<16;i++){
      Float4Bytes float4bytes;
      for(int j = 0;j<4;j++){
        float4bytes.bytes[j] = dpkt_ptr->VerticalAngle_calibration[i*4 + j];
      }
      if(float4bytes.value >= VerticalDefaultTM64[i] - 1 && float4bytes.value <= VerticalDefaultTM64[i] + 1){
        verticalTM64[i] = float4bytes.value;
      }else{
        verticalTM64[i] = VerticalDefaultTM64[i];
      }
    }

    float diff = 0;
    for(int i = 0 ; i<16;i++){
      if(i<15){
        diff = (verticalTM64[i] - verticalTM64[i + 1]) / 4; 
        VERT_ANGLE_64[0][i] = verticalTM64[i];
        for(int j = 1; j < 4; j++){
          VERT_ANGLE_64[j][i] = verticalTM64[i] - j*diff;
        }
      }else{
        VERT_ANGLE_64[0][15] = verticalTM64[15];
        for(int j = 1; j < 4; j++){
          VERT_ANGLE_64[j][15] = verticalTM64[15] - j*diff; 
        }
      }
      
    }


    for(size_t i = 0;i<4;i++){
      for(size_t j = 0; j<16 ; j++){
          this->vert_angle_list_[i*16+j] =  static_cast<int>(VERT_ANGLE_64[i][j]*100);
          // printf("vert_angle_list = %d\n",this->vert_angle_list_[i*16+j]);
      }
    }
    this->sortBeamTable();
    this->difop_flag_ = true;
  }

  if((ntohl(dpkt_ptr->IMU_Time_End) - ntohl(dpkt_ptr->IMU_Time_Start)) > 0 )
  {
    //Get Group Timestamp
    double group_timestamp = this->get_imu_time_func_(pkt);
    double imu_time_diff =(ntohl(dpkt_ptr->IMU_Time_End) - ntohl(dpkt_ptr->IMU_Time_Start)) * 1e-6 / this->lidar_const_param_.IMU_NUM_PER_GROUP;

    for(int i = 0;i < this->lidar_const_param_.IMU_NUM_PER_GROUP;i++){
        TmImu tmImu;
        tmImu.timestamp = group_timestamp + imu_time_diff * i;
        
        //IMU_Acceleration
        tmImu.imu_acceleration_x  = 9.8 * (int16_t)ntohs(dpkt_ptr->IMU_data[i].IMU_acceleration_x) * (this->lidar_const_param_.IMU_ACC_RANGE / 32768.0);
        tmImu.imu_acceleration_y  = 9.8 * (int16_t)ntohs(dpkt_ptr->IMU_data[i].IMU_acceleration_y) * (this->lidar_const_param_.IMU_ACC_RANGE / 32768.0);
        tmImu.imu_acceleration_z  = 9.8 * (int16_t)ntohs(dpkt_ptr->IMU_data[i].IMU_acceleration_z) * (this->lidar_const_param_.IMU_ACC_RANGE / 32768.0);

        //IMU_Angular_velocity
        tmImu.imu_angular_velx = (M_PI / 180)*(int16_t)ntohs(dpkt_ptr->IMU_data[i].IMU_angular_velocity_x) *(this->lidar_const_param_.IMU_GYRO_RANGE/ 32768.0);
        tmImu.imu_angular_vely = (M_PI / 180)*(int16_t)ntohs(dpkt_ptr->IMU_data[i].IMU_angular_velocity_y) *(this->lidar_const_param_.IMU_GYRO_RANGE/ 32768.0);
        tmImu.imu_angular_velz = (M_PI / 180)*(int16_t)ntohs(dpkt_ptr->IMU_data[i].IMU_angular_velocity_z) *(this->lidar_const_param_.IMU_GYRO_RANGE/ 32768.0);

        imu_vec.emplace_back(tmImu);
    }
  }


  //Constantly refreshing time
  this->gps_time_.sync_state = dpkt_ptr->GPS_state;
  this->gps_time_.sync_time.year = dpkt_ptr->gps_time.year;
  this->gps_time_.sync_time.month = dpkt_ptr->gps_time.month;
  this->gps_time_.sync_time.day = dpkt_ptr->gps_time.day;
  this->gps_time_.sync_time.hour = dpkt_ptr->gps_time.hour;
  this->gps_time_.sync_time.minute = 0;
  this->gps_time_.sync_time.second = 0;
  this->gps_time_.sync_time.ms = 0;
  this->gps_time_.sync_time.us = 0;

return TMDecoderResult::DECODE_OK;
}
}  // namespace lidar
}  // namespace timoo