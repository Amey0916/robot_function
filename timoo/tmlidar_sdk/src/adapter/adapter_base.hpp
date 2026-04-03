#pragma once
#include "utility/common.h"
#include "utility/yaml_reader.hpp"
#include "tm_driver/msg/packet_msg.h"
#include "tm_driver/msg/scan_msg.h"
#include "tm_driver/msg/imu_msg.h"
#include "msg/tm_msg/lidar_point_cloud_msg.h"

namespace timoo
{
namespace lidar
{
enum MsgSource
{
  MSG_FROM_LIDAR = 1,
  MSG_FROM_ROS_PACKET = 2,
  MSG_FROM_PCAP = 3,
  MSG_FROM_PROTO_PACKET = 4,
  MSG_FROM_PROTO_POINTCLOUD = 5,
  MSG_FROM_IMU = 6
};
enum class AdapterType
{
  DriverAdapter,
  PointCloudRosAdapter,
  PointCloudProtoAdapter,
  PacketRosAdapter,
  PacketProtoAdapter,
  CameraTriggerRosAdapter,
  ImuAdapter
};
class AdapterBase
{
public:
  typedef std::shared_ptr<AdapterBase> Ptr;
  AdapterBase() = default;
  virtual ~AdapterBase();
  virtual void init(const YAML::Node& config) = 0;
  virtual void start();
  virtual void stop();
  virtual void sendScan(const ScanMsg& msg);
  virtual void sendPacket(const PacketMsg& msg);
  virtual void sendPointCloud(const LidarPointCloudMsg& msg);
  virtual void sendImu(const ImuMsg& msg);
  virtual void sendCameraTrigger(const CameraTrigger& msg);
  virtual void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  virtual void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  virtual void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback);
  virtual void regRecvCallback(const std::function<void(const ImuMsg&)>& callback);
  virtual void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback);
  virtual void decodeScan(const ScanMsg& msg);
  virtual void decodePacket(const PacketMsg& msg);
};

inline AdapterBase::~AdapterBase()
{
  stop();
}

inline void AdapterBase::start()
{
  return;
}

inline void AdapterBase::stop()
{
  return;
}

inline void AdapterBase::sendScan(const ScanMsg& msg)
{
  return;
}

inline void AdapterBase::sendPacket(const PacketMsg& msg)
{
  return;
}


inline void AdapterBase::sendImu(const ImuMsg& msg)
{
  return;
}

inline void AdapterBase::sendPointCloud(const LidarPointCloudMsg& msg)
{
  return;
}

inline void AdapterBase::sendCameraTrigger(const CameraTrigger& msg)
{
  return;
}

inline void AdapterBase::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  return;
}


inline void AdapterBase::regRecvCallback(const std::function<void(const ImuMsg&)>& callback)
{
  return;
}

inline void AdapterBase::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  return;
}

inline void AdapterBase::regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback)
{
  return;
}

inline void AdapterBase::regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
{
  return;
}

inline void AdapterBase::decodeScan(const ScanMsg& msg)
{
  return;
}

inline void AdapterBase::decodePacket(const PacketMsg& msg)
{
  return;
}

}  // namespace lidar
}  // namespace timoo
