#include <tm_driver/driver/decoder/decoder_TM16.hpp>
#include <tm_driver/driver/decoder/decoder_TM64.hpp>
#include <tm_driver/driver/input.hpp>
#include <tm_driver/msg/packet_msg.h>
namespace timoo
{
namespace lidar
{
template <typename T_Point>
class DecoderFactory
{
public:
  DecoderFactory() = default;
  ~DecoderFactory() = default;
  static std::shared_ptr<DecoderBase<T_Point>> createDecoder(const TMDriverParam& param);

private:
  static const LidarConstantParameter getTM16ConstantParam();
  static const LidarConstantParameter getTM64ConstantParam();
};

template <typename T_Point>
inline std::shared_ptr<DecoderBase<T_Point>> DecoderFactory<T_Point>::createDecoder(const TMDriverParam& param)
{
  std::shared_ptr<DecoderBase<T_Point>> ret_ptr;
  switch (param.lidar_type)
  {
    case LidarType::TM16:
      ret_ptr = std::make_shared<DecoderTM16<T_Point>>(param.decoder_param, getTM16ConstantParam());
      break;
    case LidarType::TM64:
      ret_ptr = std::make_shared<DecoderTM64<T_Point>>(param.decoder_param, getTM64ConstantParam());
      break;
    default:
      TM_ERROR << "Wrong LiDAR Type. Please check your LiDAR Version! " << TM_REND;
      exit(-1);
  }
  // ret_ptr->loadCalibrationFile(param.angle_path);
  return ret_ptr;
}

template <typename T_Point>
inline const LidarConstantParameter DecoderFactory<T_Point>::getTM16ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0XFFEE;   //MSOP Header
  ret_param.DIFOP_ID = 0x555511115A00FFA5; //DIFOP Header
  ret_param.BLOCK_ID = 0xEEFF;  //MSOP Block Header
  ret_param.PKT_RATE = 847.711; //
  ret_param.BLOCKS_PER_PKT = 12; 
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 16;
  ret_param.DSR_TOFFSET = 3.072;  ///
  ret_param.FIRING_TOFFSET = 49.152; ///
  ret_param.RX = 0;
  ret_param.RY = 0;
  ret_param.RZ = 0;
  ret_param.IMU_NUM_PER_GROUP = 4;
  ret_param.IMU_ACC_RANGE = 6;
  ret_param.IMU_GYRO_RANGE = 1000;
  return ret_param;
}

template <typename T_Point>
inline const LidarConstantParameter DecoderFactory<T_Point>::getTM64ConstantParam()
{
  LidarConstantParameter ret_param;
  ret_param.MSOP_ID = 0XFFEE;
  ret_param.DIFOP_ID = 0x555511115A00FFA5;
  ret_param.BLOCK_ID = 0xEEFF;
  ret_param.PKT_RATE = 847.711;
  ret_param.BLOCKS_PER_PKT = 12;
  ret_param.CHANNELS_PER_BLOCK = 32;
  ret_param.LASER_NUM = 64;
  ret_param.DSR_TOFFSET = 3.072;
  ret_param.FIRING_TOFFSET = 49.152;
  ret_param.RX = 0;
  ret_param.RY = 0;
  ret_param.RZ = 0;
  ret_param.IMU_NUM_PER_GROUP = 4;
  ret_param.IMU_ACC_RANGE = 6;
  ret_param.IMU_GYRO_RANGE = 1000;
  return ret_param;
}


}  // namespace lidar
}  // namespace timoo
