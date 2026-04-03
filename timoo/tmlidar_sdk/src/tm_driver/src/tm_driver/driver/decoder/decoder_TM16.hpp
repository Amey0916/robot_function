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
} TM16MsopBlock;

union Float4Byte{
  uint8_t bytes[4];
  float value;
};

typedef struct
{
  // TMMsopHeader header;
  TM16MsopBlock blocks[12];
  uint32_t timestamp;
  uint16_t tail;
} TM16MsopPkt;

//IMU_Data
typedef struct
{
  int16_t IMU_acceleration_x;
  int16_t IMU_acceleration_y;
  int16_t IMU_acceleration_z;
  int16_t IMU_angular_velocity_x; 
  int16_t IMU_angular_velocity_y; 
  int16_t IMU_angular_velocity_z; 
} IMU_DATA;



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
} TM16DifopPkt;

#pragma pack(pop)



template <typename T_Point>
class DecoderTM16 : public DecoderBase<T_Point>
{
public:
  DecoderTM16(const TMDecoderParam& param, const LidarConstantParameter& lidar_const_param);
  TMDecoderResult decodeDifopPkt(const uint8_t* pkt);
  TMDecoderResult decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height, int& azimuth);
  TMDecoderResult decodeDifopPkt(const uint8_t* pkt, std::vector<TmImu>& imu_vec);
  double getLidarTime(const uint8_t* pkt);
  double getIMUTime(const uint8_t* pkt);
  bool compareLine(int a);
  void split1(const std::string &s, std::vector<std::string> &elems, char delim);
  void getNumberFromString(std::string msg, std::vector<int>& result);
  double imu_total_bag_diff = 0;

  private:
    int left_min_angle;                ///< minimum angle to publish
    int left_max_angle;                ///< maximum angle to publish
    int right_min_angle;                ///< minimum angle to publish
    int right_max_angle;    
    std::vector<int > hidelineNum;
};

template <typename T_Point>
void inline DecoderTM16<T_Point>::split1(const std::string &s, std::vector<std::string> &elems, char delim)
{
	std::stringstream ss;
	s.substr(0);
	ss.str(s);
	std::string iteam;
	while (std::getline(ss, iteam, ','))
	{
		elems.push_back(iteam);
	}
}

template <typename T_Point>
void inline DecoderTM16<T_Point>::getNumberFromString(std::string msg, std::vector<int>& result)
{
	std::vector<std::string> temp;
  char a ;
	split1(msg,temp,a);

	size_t num_size = temp.size();
	result.clear();
	result.resize(num_size);
	for (int i = 0; i<num_size; i++)
	{
		int num = std::atoi(temp[i].c_str());
		result[i] = num;
	}
}



template <typename T_Point>
inline DecoderTM16<T_Point>::DecoderTM16(const TMDecoderParam& param, const LidarConstantParameter& lidar_const_param)
  : DecoderBase<T_Point>(param, lidar_const_param)
{
  this->vert_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->hori_angle_list_.resize(this->lidar_const_param_.LASER_NUM);
  this->beam_ring_table_.resize(this->lidar_const_param_.LASER_NUM);
  if (this->param_.max_distance > 150.0f)
  {
    this->param_.max_distance = 150.0f;
  }
  if (this->param_.min_distance < 0.2f || this->param_.min_distance > this->param_.max_distance)
  {
    this->param_.min_distance = 0.2f;
  }

  getNumberFromString(this->param_.hidelineNumStr,hidelineNum);

   if (this->param_.left_min_angle<0 || this->param_.left_min_angle > 360 || this->param_.left_max_angle<0||this->param_.left_min_angle>360||this->param_.left_min_angle == this->param_.left_max_angle)
  {
    //avoid returning empty cloud if min_angle = max_angle
    this->left_min_angle = 0;
    this->left_max_angle = 36000;
  }else{
    this->left_min_angle  = this->param_.left_min_angle *100 ;
    this->left_max_angle = this->param_.left_max_angle * 100 ;
  }
    if (this->param_.right_min_angle<0 || this->param_.right_min_angle > 360 || this->param_.right_max_angle<0||this->param_.right_min_angle>360||this->param_.right_min_angle == this->param_.right_max_angle)
  {
    //avoid returning empty cloud if min_angle = max_angle
    this->right_min_angle = 0;
    this->right_max_angle = 36000;
  }else{
    this->right_min_angle  = this->param_.right_min_angle *100 ;
    this->right_max_angle = this->param_.right_max_angle * 100 ;
  }

  printf("driver_param.decoder_param.left_min_angle:%d \n",this->left_min_angle);
  printf("driver_param.decoder_param.left_min_angle:%d \n",this->left_max_angle);
  printf("driver_param.decoder_param.left_min_angle:%d \n",this->right_min_angle);
  printf("driver_param.decoder_param.left_min_angle:%d \n",this->right_max_angle);
  printf("cut_start_angle:%d \n",this->param_.cut_start_angle *100);
  printf("cut_end_angle:%d \n",this->param_.cut_end_angle *100);
}

template <typename T_Point>
inline double DecoderTM16<T_Point>::getLidarTime(const uint8_t* pkt)
{
  return this->template calculateTimeYMD<TM16MsopPkt>(pkt);
}

template <typename T_Point>
inline double DecoderTM16<T_Point>::getIMUTime(const uint8_t* pkt)
{
  return this->template calculateIMUTimeYMD<TM16DifopPkt>(pkt);
}

template <typename T_Point>
inline bool DecoderTM16<T_Point>::compareLine(int a)
{
    // printf("num == %d\n",hidelineNum.size());
    for(int i = 0 ; i < this->hidelineNum.size();i++)
    {      
      
      if(a == this->hidelineNum[i])
      return true;
    }
      return false;
}


template <typename T_Point>
inline TMDecoderResult DecoderTM16<T_Point>::decodeMsopPkt(const uint8_t* pkt, std::vector<T_Point>& vec, int& height,
                                                           int& azimuth)
{
  height = this->lidar_const_param_.LASER_NUM;
  const TM16MsopPkt* mpkt_ptr = reinterpret_cast<const TM16MsopPkt*>(pkt);
  //Add judgment on packet headers
  // if (mpkt_ptr->header.id != this->lidar_const_param_.MSOP_ID)
  // {
  //   return TMDecoderResult::WRONG_PKT_HEADER;
  // }
  azimuth = (mpkt_ptr->blocks[0].azimuth);
  // this->current_temperature_ = this->computeTemperature(mpkt_ptr->header.temp_raw);
  
  double block_timestamp = this->get_point_time_func_(pkt);
   
  // this->check_camera_trigger_func_(azimuth, pkt);
  float azi_diff = 0;
  for (size_t blk_idx = 0; blk_idx < this->lidar_const_param_.BLOCKS_PER_PKT; blk_idx++)
  {
    if (mpkt_ptr->blocks[blk_idx].id != this->lidar_const_param_.BLOCK_ID)
    {
       break;
    }
    int cur_azi  = (mpkt_ptr->blocks[blk_idx].azimuth);;
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
     
     // int azi_channel_final = this->azimuthCalibration(azi_channel_ori, channel_idx % 16);
      int azi_channel_final = azi_channel_ori;

      float distance = (mpkt_ptr->blocks[blk_idx].channels[channel_idx].distance) * TM_DIS_RESOLUTION;
      int angle_horiz_ori = static_cast<int>(azi_channel_ori + TM_ONE_ROUND) % TM_ONE_ROUND;
      int angle_vert = ((this->vert_angle_list_[channel_idx % 16]) + TM_ONE_ROUND) % TM_ONE_ROUND;
      

      int ring_now = this->beam_ring_table_[channel_idx % 16];
      


      T_Point point;
      // if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
      //     ((this->angle_flag_ && azi_channel_final >= this->start_angle_ && azi_channel_final <= this->end_angle_) ||
      //      (!this->angle_flag_ &&
      //       ((azi_channel_final >= this->start_angle_) || (azi_channel_final <= this->end_angle_)))))

       if ((distance <= this->param_.max_distance && distance >= this->param_.min_distance) &&
       		(angle_horiz_ori <= this->param_.cut_start_angle * 100  || angle_horiz_ori >= this->param_.cut_end_angle * 100 ) &&
          (
            !compareLine(ring_now) || (
               compareLine(ring_now) &&(((
                 angle_horiz_ori >= left_min_angle 
               && angle_horiz_ori <= left_max_angle 
               && left_min_angle < left_max_angle)
               ||(left_min_angle > left_max_angle 
               && (angle_horiz_ori <= left_max_angle 
               || angle_horiz_ori >= left_min_angle)) )||
             ((angle_horiz_ori >= right_min_angle 
               && angle_horiz_ori <= right_max_angle 
               && right_min_angle < right_max_angle)
               ||(right_min_angle > right_max_angle 
               && (angle_horiz_ori <= right_max_angle 
               || angle_horiz_ori >= right_min_angle)))))

          ))
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
      setRing(point, this->beam_ring_table_[channel_idx % 16]);

      double timestamp = block_timestamp + channel_idx * this->time_duration_between_firing_ * 1e-6;
      setTimestamp(point, timestamp);

      vec.emplace_back(std::move(point));
    }
  }
  return TMDecoderResult::DECODE_OK;
}

template <typename T_Point>
inline TMDecoderResult DecoderTM16<T_Point>::decodeDifopPkt(const uint8_t* pkt, std::vector<TmImu>& imu_vec){
  const TM16DifopPkt* dpkt_ptr = reinterpret_cast<const TM16DifopPkt*>(pkt);
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return TMDecoderResult::WRONG_PKT_HEADER;
  }
  // int16_t imu_tempdata;
  //  //IMU_Temp
  // imu_tempdata = (int16_t)ntohs(dpkt_ptr->IMU_Temp) / 32;
  // if(imu_tempdata > 1023){
  //   this->tm_imu_.IMU_TEMP = (imu_tempdata -2048) * 0.125 + 23;
  // }else{
  //   this->tm_imu_.IMU_TEMP = imu_tempdata * 0.125 + 23;
  // }

  this->template decodeDifopCommon<TM16DifopPkt>(pkt, LidarType::TM16);
  if (!this->difop_flag_)
  {  
    float ChannelToAngleValueTM16[16] = {-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15};

    float AngleNoToChannelC16[16] = {0,2,4,6,8,10,12,14,1,3,5,7,9,11,13,15};
    
    for(int i = 0; i < 16 ; i++)
    {
      Float4Byte float4byte;
      for(int j = 0 ; j < 4 ; j++){
        float4byte.bytes[j] = dpkt_ptr->VerticalAngle_calibration[i*4 + j];
      }
      if(float4byte.value >= -16 + i*2 && float4byte.value <= -14 + i*2){
        this->vert_angle_list_[AngleNoToChannelC16[i]] = float4byte.value * 100;
      }
      else{
        this->vert_angle_list_[i] = ChannelToAngleValueTM16[i] * 100;
      }
    }
  //  this->hori_angle_list_[i] = 0;
    this->sortBeamTable();
    this->difop_flag_ = true;
  }

  if((ntohl(dpkt_ptr->IMU_Time_End) - ntohl(dpkt_ptr->IMU_Time_Start)) > 0){
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

template <typename T_Point>
inline TMDecoderResult DecoderTM16<T_Point>::decodeDifopPkt(const uint8_t* pkt)
{
  const TM16DifopPkt* dpkt_ptr = reinterpret_cast<const TM16DifopPkt*>(pkt);
  if (dpkt_ptr->id != this->lidar_const_param_.DIFOP_ID)
  {
    return TMDecoderResult::WRONG_PKT_HEADER;
  }
  this->template decodeDifopCommon<TM16DifopPkt>(pkt, LidarType::TM16);
  if (!this->difop_flag_)
  {
    float ChannelToAngleValueTM16[16] = {-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15};

    int AngleNoToChannelC16[16] = {0,2,4,6,8,10,12,14,1,3,5,7,9,11,13,15};
    
    for(int i = 0; i < 16; i++)
    {
      Float4Byte float4byte;
      for(int j = 0 ; j < 4 ; j++){
        float4byte.bytes[j] = dpkt_ptr->VerticalAngle_calibration[i*4 + j];
      }
      if(float4byte.value >= -16 + i*2 && float4byte.value <= -14 + i*2){
        this->vert_angle_list_[AngleNoToChannelC16[i]] = float4byte.value * 100;    
      }
      else
      {
        this->vert_angle_list_[i] = ChannelToAngleValueTM16[i] * 100;
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

}  // namespace lidar
}  // namespace timoo
