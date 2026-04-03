#pragma once
#include <string>
#include <array>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
struct TmPointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(TmPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
                                                     uint16_t, ring, ring)(double, timestamp, timestamp))
#ifdef POINT_TYPE_XYZI
typedef pcl::PointXYZI PointT;
#elif POINT_TYPE_XYZIRT

typedef TmPointXYZIRT PointT;
#endif

namespace timoo
{
namespace lidar
{
/**
 * @brief Point cloud message for Timoo SDK.
 * @detail Timoo LidarPointCloudMsg is defined for passing lidar point cloud accross different modules
 *         If ROS is turned on , we provide translation functions between ROS message and Timoo message
 *         If Proto is turned on , we provide translation functions between Protobuf message and Timoo message
 */

struct alignas(16) LidarPointCloudMsg
{
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string frame_id = "";

  PointCloudConstPtr point_cloud_ptr;  ///< the point cloud pointer

  LidarPointCloudMsg() = default;
  LidarPointCloudMsg(const PointCloudPtr& ptr) : point_cloud_ptr(ptr)
  {
  }
  typedef std::shared_ptr<LidarPointCloudMsg> Ptr;
  typedef std::shared_ptr<const LidarPointCloudMsg> ConstPtr;
};

}  // namespace lidar
}  // namespace timoo
