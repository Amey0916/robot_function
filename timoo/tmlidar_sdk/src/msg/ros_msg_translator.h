#pragma once
#ifdef ROS_FOUND
#include <std_msgs/Time.h>
#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// #include <sensor_msgs/Imu.msg>
// #include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/Imu.h>
#include "tm_driver/src/tm_driver/msg/imu_msg.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include "msg/tm_msg/lidar_point_cloud_msg.h"
#include "msg/ros_msg/lidar_packet_ros.h"
#include "msg/ros_msg/lidar_scan_ros.h"
#include "tm_driver/msg/packet_msg.h"
#include "tm_driver/msg/scan_msg.h"
#include "tm_driver/driver/driver_param.h"


namespace timoo
{
namespace lidar
{
/************************************************************************/
/**Translation functions between Timoo message and ROS message**/
/************************************************************************/
inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointCloudMsg& tm_msg)
{
  sensor_msgs::PointCloud2 ros_msg;
  pcl::toROSMsg(*tm_msg.point_cloud_ptr, ros_msg);
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(tm_msg.timestamp);
  ros_msg.header.frame_id = tm_msg.frame_id;
  ros_msg.header.seq = tm_msg.seq;
  return std::move(ros_msg);
}

inline sensor_msgs::Imu toRosMsg(const ImuMsg& msg,int i)
{
  TmImu tm_imu = (*msg.imu_vec_ptr)[i];
  sensor_msgs::Imu ros_msg;
  ros_msg.header.stamp.sec = (uint32_t)floor(tm_imu.timestamp);
  ros_msg.header.stamp.nsec = (uint32_t)round((tm_imu.timestamp - ros_msg.header.stamp.sec) * 1e9);
  // ros_msg.header.stamp = ros_msg.header.stamp.fromSec(tm_imu.timestamp);
  // ros_msg.header.stamp.nsec = ros_msg.header.stamp.fromnSec((tm_imu.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = msg.frame_id;
  ros_msg.linear_acceleration.x = tm_imu.imu_acceleration_x;
  ros_msg.linear_acceleration.y = tm_imu.imu_acceleration_y;
  ros_msg.linear_acceleration.z = tm_imu.imu_acceleration_z;
  ros_msg.angular_velocity.x  = tm_imu.imu_angular_velx;
  ros_msg.angular_velocity.y  = tm_imu.imu_angular_vely;
  ros_msg.angular_velocity.z  = tm_imu.imu_angular_velz;
  return std::move(ros_msg);
}

inline PacketMsg toTmMsg(const LidarType& lidar_type, const PktType& pkt_type,
                         const tmlidar_msgs::tmlidarPacket& ros_msg)
{
  size_t pkt_length;
  switch (lidar_type)
  {
    default:
      pkt_length = MECH_PKT_LEN;
      break;
  }
  PacketMsg tm_msg(pkt_length);
  for (size_t i = 0; i < pkt_length; i++)
  {
    tm_msg.packet[i] = ros_msg.data[i];
  }
  return std::move(tm_msg);
}
inline tmlidar_msgs::tmlidarPacket toRosMsg(const PacketMsg& tm_msg)
{
  tmlidar_msgs::tmlidarPacket ros_msg;
  for (size_t i = 0; i < tm_msg.packet.size(); i++)
  {
    ros_msg.data[i] = tm_msg.packet[i];
  }
  return std::move(ros_msg);
}
inline ScanMsg toTmMsg(const LidarType& lidar_type, const PktType& pkt_type, const tmlidar_msgs::tmlidarScan& ros_msg)
{
  ScanMsg tm_msg;
  tm_msg.seq = ros_msg.header.seq;
  tm_msg.timestamp = ros_msg.header.stamp.toSec();
  tm_msg.frame_id = ros_msg.header.frame_id;

  for (uint32_t i = 0; i < ros_msg.packets.size(); i++)
  {
    PacketMsg tmp = toTmMsg(lidar_type, pkt_type, ros_msg.packets[i]);
    tm_msg.packets.emplace_back(std::move(tmp));
  }
  return std::move(tm_msg);
}
inline tmlidar_msgs::tmlidarScan toRosMsg(const ScanMsg& tm_msg)
{
  tmlidar_msgs::tmlidarScan ros_msg;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(tm_msg.timestamp);
  ros_msg.header.frame_id = tm_msg.frame_id;
  ros_msg.header.seq = tm_msg.seq;
  for (uint32_t i = 0; i < tm_msg.packets.size(); i++)
  {
    tmlidar_msgs::tmlidarPacket tmp = toRosMsg(tm_msg.packets[i]);
    ros_msg.packets.emplace_back(std::move(tmp));
  }
  return std::move(ros_msg);
}
inline std_msgs::Time toRosMsg(const CameraTrigger& tm_msg)
{
  std_msgs::Time ros_msg;
  ros_msg.data = ros_msg.data.fromSec(tm_msg.second);
  return ros_msg;
}

}  // namespace lidar
}  // namespace timoo
#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sensor_msgs/msg/nav_sat_fix.h"
#include "tmlidar_msg/msg/tmlidar_packet.hpp"
#include "tmlidar_msg/msg/tmlidar_scan.hpp"
#include "tm_driver/src/tm_driver/msg/imu_msg.h"
namespace timoo
{
namespace lidar
{
/************************************************************************/
/**Translation functions between Timoo message and ROS2 message**/
/************************************************************************/
inline sensor_msgs::msg::PointCloud2 toRosMsg(const LidarPointCloudMsg& tm_msg)
{
  sensor_msgs::msg::PointCloud2 ros_msg;
  pcl::toROSMsg(*tm_msg.point_cloud_ptr, ros_msg);
  ros_msg.header.stamp.sec = (uint32_t)floor(tm_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((tm_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = tm_msg.frame_id;
  return std::move(ros_msg);
}

inline sensor_msgs::msg::Imu toRosMsg(const ImuMsg& msg,int i)
{
  TmImu tm_imu = (*msg.imu_vec_ptr)[i];
  sensor_msgs::msg::Imu ros_msg;
  ros_msg.header.stamp.sec = (uint32_t)floor(tm_imu.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((tm_imu.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = msg.frame_id;
  ros_msg.linear_acceleration.x = tm_imu.imu_acceleration_x;
  ros_msg.linear_acceleration.y = tm_imu.imu_acceleration_y;
  ros_msg.linear_acceleration.z = tm_imu.imu_acceleration_z;
  ros_msg.angular_velocity.x  = tm_imu.imu_angular_velx;
  ros_msg.angular_velocity.y  = tm_imu.imu_angular_vely;
  ros_msg.angular_velocity.z  = tm_imu.imu_angular_velz;
  return std::move(ros_msg);
}

inline PacketMsg toTmMsg(const LidarType& lidar_type, const PktType& pkt_type,
                         const tmlidar_msg::msg::TmlidarPacket& ros_msg)
{
  size_t pkt_length;
  switch (lidar_type)
  {
    default:
      pkt_length = MECH_PKT_LEN;
      break;
  }
  PacketMsg tm_msg(pkt_length);
  for (size_t i = 0; i < pkt_length; i++)
  {
    tm_msg.packet[i] = ros_msg.data[i];
  }
  return std::move(tm_msg);
}
inline tmlidar_msg::msg::TmlidarPacket toRosMsg(const PacketMsg& tm_msg)
{
  tmlidar_msg::msg::TmlidarPacket ros_msg;
  for (size_t i = 0; i < tm_msg.packet.size(); i++)
  {
    ros_msg.data[i] = tm_msg.packet[i];
  }
  return std::move(ros_msg);
}
inline ScanMsg toTmMsg(const LidarType& lidar_type, const PktType& pkt_type,
                       const tmlidar_msg::msg::TmlidarScan& ros_msg)
{
  ScanMsg tm_msg;
  tm_msg.timestamp = ros_msg.header.stamp.sec + double(ros_msg.header.stamp.nanosec) / 1e9;
  tm_msg.frame_id = ros_msg.header.frame_id;
  for (uint32_t i = 0; i < ros_msg.packets.size(); i++)
  {
    PacketMsg tmp = toTmMsg(lidar_type, pkt_type, ros_msg.packets[i]);
    tm_msg.packets.emplace_back(std::move(tmp));
  }
  return tm_msg;
}
inline tmlidar_msg::msg::TmlidarScan toRosMsg(const ScanMsg& tm_msg)
{
  tmlidar_msg::msg::TmlidarScan ros_msg;
  ros_msg.header.stamp.sec = (uint32_t)floor(tm_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((tm_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = tm_msg.frame_id;
  for (uint32_t i = 0; i < tm_msg.packets.size(); i++)
  {
    tmlidar_msg::msg::TmlidarPacket tmp = toRosMsg(tm_msg.packets[i]);
    ros_msg.packets.emplace_back(std::move(tmp));
  }
  return ros_msg;
}
}  // namespace lidar
}  // namespace timoo
#endif  // ROS2_FOUND