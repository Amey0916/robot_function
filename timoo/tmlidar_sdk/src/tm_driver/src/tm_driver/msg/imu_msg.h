#pragma once
#include <string>
#include <array>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <tm_driver/common/common_header.h>

struct TmImu
{
    double timestamp=0;
    float imu_acceleration_x = 0.0;
    float imu_acceleration_y = 0.0;
    float imu_acceleration_z = 0.0;
    float imu_angular_velx = 0.0;
    float imu_angular_vely = 0.0;
    float imu_angular_velz = 0.0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;


namespace timoo
{
namespace lidar
{
#ifdef _MSC_VER
struct __declspec(align(16)) ImuMsg
#elif __GNUC__
struct __attribute__((aligned(16))) ImuMsg  ///< LiDAR single packet message
#endif    
{
    typedef std::vector<TmImu> ImuVec;
    typedef std::shared_ptr<ImuVec> ImuVecPtr;
    double imu_timestamp = 0.0;
    std::string frame_id = "";      ///< Imu frame id
    uint32_t seq = 0;               ///< Sequence number of message
    int16_t imu_temp = 0;

    ImuVecPtr imu_vec_ptr;
    ImuMsg() = default;
      explicit ImuMsg(const ImuVecPtr& ptr) : imu_vec_ptr(ptr)
  {
  }
  typedef std::shared_ptr<ImuMsg> ImuPtr;
  typedef std::shared_ptr<const ImuMsg> ConstImuPtr;
};
}        
}