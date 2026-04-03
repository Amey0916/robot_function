#pragma once
#ifdef PROTO_FOUND
#include "msg/tm_msg/lidar_point_cloud_msg.h"
#include "tm_driver/msg/packet_msg.h"
#include "tm_driver/msg/scan_msg.h"
#include "msg/proto_msg/point_cloud.pb.h"
#include "msg/proto_msg/packet.pb.h"
#include "msg/proto_msg/scan.pb.h"
namespace timoo
{
namespace lidar
{
/************************************************************************/
/**Translation functions between Timoo message and protobuf message**/
/************************************************************************/
inline proto_msg::LidarPointCloud toProtoMsg(const LidarPointCloudMsg& tm_msg)
{
  proto_msg::LidarPointCloud proto_msg;
  proto_msg.set_timestamp(tm_msg.timestamp);
  proto_msg.set_seq(tm_msg.seq);
  proto_msg.set_frame_id(tm_msg.frame_id);
  proto_msg.set_height(tm_msg.point_cloud_ptr->height);
  proto_msg.set_width(tm_msg.point_cloud_ptr->width);
  proto_msg.set_is_dense(tm_msg.point_cloud_ptr->is_dense);

  for (size_t i = 0; i < tm_msg.point_cloud_ptr->size(); i++)
  {
    proto_msg.add_data(tm_msg.point_cloud_ptr->points[i].x);
    proto_msg.add_data(tm_msg.point_cloud_ptr->points[i].y);
    proto_msg.add_data(tm_msg.point_cloud_ptr->points[i].z);
    proto_msg.add_data(tm_msg.point_cloud_ptr->points[i].intensity);
  }

  return std::move(proto_msg);
}

inline LidarPointCloudMsg toTmMsg(const proto_msg::LidarPointCloud& proto_msg)
{
  LidarPointCloudMsg tm_msg;
  tm_msg.timestamp = proto_msg.timestamp();
  tm_msg.seq = proto_msg.seq();
  tm_msg.frame_id = proto_msg.frame_id();
  LidarPointCloudMsg::PointCloud* ptr_tmp = new LidarPointCloudMsg::PointCloud();
  for (int i = 0; i < proto_msg.data_size(); i += 4)
  {
    PointT point;
    point.x = proto_msg.data(i);
    point.y = proto_msg.data(i + 1);
    point.z = proto_msg.data(i + 2);
    point.intensity = proto_msg.data(i + 3);
    ptr_tmp->push_back(point);
  }
  ptr_tmp->height = proto_msg.height();
  ptr_tmp->width = proto_msg.width();
  ptr_tmp->is_dense = proto_msg.is_dense();
  tm_msg.point_cloud_ptr.reset(ptr_tmp);
  return tm_msg;
}

inline proto_msg::LidarScan toProtoMsg(const ScanMsg& tm_msg)
{
  proto_msg::LidarScan proto_msg;
  proto_msg.set_timestamp(tm_msg.timestamp);
  proto_msg.set_seq(tm_msg.seq);
  for (auto iter : tm_msg.packets)
  {
    void* data_ptr = malloc(iter.packet.size());
    memcpy(data_ptr, iter.packet.data(), iter.packet.size());
    proto_msg.add_data(data_ptr, iter.packet.size());
    free(data_ptr);
  }
  return proto_msg;
}

inline ScanMsg toTmMsg(const proto_msg::LidarScan& proto_msg)
{
  ScanMsg tm_msg;
  tm_msg.timestamp = proto_msg.timestamp();
  tm_msg.seq = proto_msg.seq();
  for (int i = 0; i < proto_msg.data_size(); i++)
  {
    std::string data_str = proto_msg.data(i);
    PacketMsg tmp_pkt(data_str.size());
    memcpy(tmp_pkt.packet.data(), data_str.data(), data_str.size());
    tm_msg.packets.emplace_back(std::move(tmp_pkt));
  }
  return tm_msg;
}

inline proto_msg::LidarPacket toProtoMsg(const PacketMsg& tm_msg)
{
  proto_msg::LidarPacket proto_msg;
  void* data_ptr = malloc(tm_msg.packet.size());
  memcpy(data_ptr, tm_msg.packet.data(), tm_msg.packet.size());
  proto_msg.set_data(data_ptr, tm_msg.packet.size());
  free(data_ptr);
  return proto_msg;
}

inline PacketMsg toTmMsg(const proto_msg::LidarPacket& proto_msg)
{
  std::string data_str = proto_msg.data();
  PacketMsg tm_msg(data_str.size());
  memcpy(tm_msg.packet.data(), data_str.data(), data_str.size());
  return tm_msg;
}

}  // namespace lidar
}  // namespace timoo
#endif  // PROTO_FOUND
