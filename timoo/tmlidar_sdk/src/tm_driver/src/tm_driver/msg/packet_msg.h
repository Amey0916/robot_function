#pragma once
#include <tm_driver/common/common_header.h>

namespace timoo
{
namespace lidar
{
enum PktType
{
  MSOP = 0,
  DIFOP
};
#ifdef _MSC_VER
struct __declspec(align(16)) PacketMsg
#elif __GNUC__
struct __attribute__((aligned(16))) PacketMsg  ///< LiDAR single packet message
#endif
{
  std::vector<uint8_t> packet;
  PacketMsg()
  {
  }
  PacketMsg(const PacketMsg& msg)
  {
    this->packet.assign(msg.packet.begin(), msg.packet.end());
  }
  PacketMsg(const size_t& pkt_length)
  {
    packet.resize(pkt_length);
  }
};
}  // namespace lidar
}  // namespace timoo
