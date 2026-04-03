#pragma once
#ifdef ROS_FOUND
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"
#include <ros/ros.h>
#include <ros/publisher.h>

namespace timoo
{
namespace lidar
{
class CameraTriggerRosAdapter : virtual public AdapterBase
{
public:
  CameraTriggerRosAdapter() = default;
  ~CameraTriggerRosAdapter() = default;
  void init(const YAML::Node& config);
  void sendCameraTrigger(const CameraTrigger& msg);

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::map<std::string, ros::Publisher> pub_map_;
};

inline void CameraTriggerRosAdapter::init(const YAML::Node& config)
{
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  if (config["camera"] && config["camera"].Type() != YAML::NodeType::Null)
  {
    for (auto iter : config["camera"])
    {
      std::string frame_id;
      yamlRead<std::string>(iter, "frame_id", frame_id, "tm_camera");
      ros::Publisher pub = nh_->advertise<std_msgs::Time>(frame_id + "_trigger", 10);
      pub_map_.emplace(frame_id, pub);
    }
  }
}

inline void CameraTriggerRosAdapter::sendCameraTrigger(const CameraTrigger& msg)
{
  auto iter = pub_map_.find(msg.first);
  if (iter != pub_map_.end())
  {
    iter->second.publish(toRosMsg(msg));
  }
}

}  // namespace lidar
}  // namespace timoo
#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"
#include "rclcpp/rclcpp.hpp"
namespace timoo
{
namespace lidar
{
///< Add in the future
}  // namespace lidar
}  // namespace timoo
#endif  // ROS2_FOUND