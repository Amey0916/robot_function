#pragma once
#ifdef ROS_FOUND
#include <ros/ros.h>
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"

namespace timoo
{
namespace lidar
{
class PacketRosAdapter : virtual public AdapterBase
{
public:
  PacketRosAdapter() = default;
  ~PacketRosAdapter() = default;
  void init(const YAML::Node& config);
  void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  void sendScan(const ScanMsg& msg);
  void sendPacket(const PacketMsg& msg);

private:
  void localMsopCallback(const tmlidar_msgs::tmlidarScan& msg);
  void localDifopCallback(const tmlidar_msgs::tmlidarPacket& msg);

private:
  std::unique_ptr<ros::NodeHandle> nh_;
  std::vector<std::function<void(const ScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> packet_cb_vec_;
  ros::Publisher scan_pub_;
  ros::Publisher packet_pub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber packet_sub_;
  LidarType lidar_type_;
};

inline void PacketRosAdapter::init(const YAML::Node& config)
{
  int msg_source;
  bool send_packet_ros;
  std::string ros_recv_topic;
  std::string ros_send_topic;
  std::string lidar_type_str;
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  yamlReadAbort<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_packet_ros", send_packet_ros, false);
  yamlRead<std::string>(config["driver"], "lidar_type", lidar_type_str, "TM16");
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", ros_recv_topic, "tmlidar_packets");
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", ros_send_topic, "tmlidar_packets");
  lidar_type_ = TMDriverParam::strToLidarType(lidar_type_str);
  if (msg_source == MsgSource::MSG_FROM_ROS_PACKET)
  {
    packet_sub_ = nh_->subscribe(ros_recv_topic + "_difop", 1, &PacketRosAdapter::localDifopCallback, this);
    scan_sub_ = nh_->subscribe(ros_recv_topic, 1, &PacketRosAdapter::localMsopCallback, this);
    send_packet_ros = false;
  }
  if (send_packet_ros)
  {
    packet_pub_ = nh_->advertise<tmlidar_msgs::tmlidarPacket>(ros_send_topic + "_difop", 10);
    scan_pub_ = nh_->advertise<tmlidar_msgs::tmlidarScan>(ros_send_topic, 10);
  }
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  scan_cb_vec_.emplace_back(callback);
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  packet_cb_vec_.emplace_back(callback);
}

inline void PacketRosAdapter::sendScan(const ScanMsg& msg)
{
  scan_pub_.publish(toRosMsg(msg));
}

inline void PacketRosAdapter::sendPacket(const PacketMsg& msg)
{
  packet_pub_.publish(toRosMsg(msg));
}

inline void PacketRosAdapter::localMsopCallback(const tmlidar_msgs::tmlidarScan& msg)
{
  for (auto& cb : scan_cb_vec_)
  {
    cb(toTmMsg(lidar_type_, PktType::MSOP, msg));
  }
}

inline void PacketRosAdapter::localDifopCallback(const tmlidar_msgs::tmlidarPacket& msg)
{
  for (auto& cb : packet_cb_vec_)
  {
    cb(toTmMsg(lidar_type_, PktType::DIFOP, msg));
  }
}

}  // namespace lidar
}  // namespace timoo
#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"
namespace timoo
{
namespace lidar
{
class PacketRosAdapter : virtual public AdapterBase
{
public:
  PacketRosAdapter() = default;
  ~PacketRosAdapter();
  void init(const YAML::Node& config);
  void start();
  void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  void sendScan(const ScanMsg& msg);
  void sendPacket(const PacketMsg& msg);

private:
  void localMsopCallback(const tmlidar_msg::msg::TmlidarScan::SharedPtr msg);
  void localDifopCallback(const tmlidar_msg::msg::TmlidarPacket::SharedPtr msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<tmlidar_msg::msg::TmlidarScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<tmlidar_msg::msg::TmlidarPacket>::SharedPtr packet_pub_;
  rclcpp::Subscription<tmlidar_msg::msg::TmlidarScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<tmlidar_msg::msg::TmlidarPacket>::SharedPtr packet_sub_;
  std::vector<std::function<void(const ScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> packet_cb_vec_;
  LidarType lidar_type_;
};
inline PacketRosAdapter::~PacketRosAdapter()
{
  stop();
}

inline void PacketRosAdapter::init(const YAML::Node& config)
{
  int msg_source;
  bool send_packet_ros;
  std::string ros_recv_topic;
  std::string ros_send_topic;
  std::string lidar_type_str;
  node_ptr_.reset(new rclcpp::Node("tmlidar_packets_adapter"));
  yamlReadAbort<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_packet_ros", send_packet_ros, false);
  yamlRead<std::string>(config["driver"], "lidar_type", lidar_type_str, "TM16");
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", ros_recv_topic, "tmlidar_packets");
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", ros_send_topic, "tmlidar_packets");
  lidar_type_ = TMDriverParam::strToLidarType(lidar_type_str);
  if (msg_source == MsgSource::MSG_FROM_ROS_PACKET)
  {
    packet_sub_ = node_ptr_->create_subscription<tmlidar_msg::msg::TmlidarPacket>(
        ros_recv_topic + "_difop", 10,
        [this](const tmlidar_msg::msg::TmlidarPacket::SharedPtr msg) { localDifopCallback(msg); });
    scan_sub_ = node_ptr_->create_subscription<tmlidar_msg::msg::TmlidarScan>(
        ros_recv_topic, 10, [this](const tmlidar_msg::msg::TmlidarScan::SharedPtr msg) { localMsopCallback(msg); });
  }
  if (send_packet_ros)
  {
    packet_pub_ = node_ptr_->create_publisher<tmlidar_msg::msg::TmlidarPacket>(ros_send_topic + "_difop", 10);
    scan_pub_ = node_ptr_->create_publisher<tmlidar_msg::msg::TmlidarScan>(ros_send_topic, 10);
  }
}

inline void PacketRosAdapter::start()
{
  std::thread t([this]() { rclcpp::spin(node_ptr_); });
  t.detach();
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  scan_cb_vec_.emplace_back(callback);
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  packet_cb_vec_.emplace_back(callback);
}

inline void PacketRosAdapter::sendScan(const ScanMsg& msg)
{
  scan_pub_->publish(toRosMsg(msg));
}

inline void PacketRosAdapter::sendPacket(const PacketMsg& msg)
{
  packet_pub_->publish(toRosMsg(msg));
}

inline void PacketRosAdapter::localMsopCallback(const tmlidar_msg::msg::TmlidarScan::SharedPtr msg)
{
  for (auto& cb : scan_cb_vec_)
  {
    cb(toTmMsg(lidar_type_, PktType::MSOP, *msg));
  }
}

inline void PacketRosAdapter::localDifopCallback(const tmlidar_msg::msg::TmlidarPacket::SharedPtr msg)
{
  for (auto& cb : packet_cb_vec_)
  {
    cb(toTmMsg(lidar_type_, PktType::DIFOP, *msg));
  }
}
}  // namespace lidar
}  // namespace timoo
#endif  // ROS2_FOUND