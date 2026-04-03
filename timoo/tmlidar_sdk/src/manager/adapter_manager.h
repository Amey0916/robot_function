/********************************************************************************************************************
 * Basic struct:
 *
 *
 *                                              AdapterBase
 *                                            /      |      \
 *                                           /       |       \
 *     ROS:                        PacketRosAdapter  |   PointCloudRosAdapter
 *     Protobuf:          PacketProtoAdapter         |            PointCloudProtoAdapter
 *     Driver:                       \         DriverAdapter           /
 *                                    \        /           \          /
 *                                     \      /             \        /
 *                                      \    /               \      /
 *                                      Packet              PointCloud
 *
 *
 * AdapterManager:
 *
 * step1
 *                  -msg_source=1 -> packet receiver: DriverAdapter; point cloud receiver: DriverAdapter
 *                  -msg_source=2 -> packet receiver: PacketRosAdapter; point cloud receiver: DriverAdapter
 * createReceiver ->-msg_source=3 -> packet receiver: DriverAdapter; point cloud receiver: DriverAdapter
 *                  -msg_source=4 -> packet receiver: PacketProtoAdapter; point cloud receiver: DriverAdapter
 *                  -msg_source=5 -> packet receiver: None; point cloud receiver: PointCloudProtoAdapter
 *
 * step2
 *
 *                      -send_packet_ros=true -> PacketRosAdapter
 * createTransmitter -> -send_point_cloud_ros=true -> PointCloudRosAdapter
 *                      -send_packet_proto=true -> PacketProtoAdapter
 *                      -send_point_cloud_proto=true -> PointCloudProtoAdapter
 *
 * step3
 * Register the transmitter's sending functions into the receiver
 *
 * step4
 * Start all the receivers
 *
 * ******************************************************************************************************************/

#pragma once
#include "utility/yaml_reader.hpp"
#include "adapter/driver_adapter.hpp"
#include "adapter/packet_ros_adapter.hpp"
#include "adapter/point_cloud_ros_adapter.hpp"
#include "adapter/packet_protobuf_adapter.hpp"
#include "adapter/point_cloud_protobuf_adapter.hpp"
#include "adapter/camera_trigger_adapter.hpp"
#include "adapter/imu_ros_adapter.hpp"

namespace timoo
{
namespace lidar
{
class AdapterManager
{
public:
  AdapterManager() = default;
  ~AdapterManager();
  void init(const YAML::Node& config);
  void start();
  void stop();

private:
  std::shared_ptr<AdapterBase> createReceiver(const YAML::Node& config, const AdapterType& adapter_type);
  std::shared_ptr<AdapterBase> createTransmitter(const YAML::Node& config, const AdapterType& adapter_type);

private:
  bool packet_thread_flag_;
  bool point_cloud_thread_flag_;
  std::vector<AdapterBase::Ptr> packet_receive_adapter_vec_;
  std::vector<AdapterBase::Ptr> point_cloud_receive_adapter_vec_;
  std::vector<AdapterBase::Ptr> packet_transmit_adapter_vec_;
  std::vector<AdapterBase::Ptr> point_cloud_transmit_adapter_vec_;
  std::vector<AdapterBase::Ptr> imu_receive_adapter_vec_;
  std::vector<AdapterBase::Ptr> imu_transmit_adapter_vec_;

};
}  // namespace lidar
}  // namespace timoo
