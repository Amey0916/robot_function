#pragma once
#ifdef ROS_FOUND
#include <ros/ros.h>
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"

namespace timoo
{
namespace lidar
{
class ImuAdapter : virtual public AdapterBase
{
public:
  ImuAdapter() = default;
  ~ImuAdapter() = default;
  void init(const YAML::Node& config);
  void sendImu(const ImuMsg& msg);

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher imu_pub_;
};

inline void ImuAdapter::init(const YAML::Node& config)
{
  bool send_imu_ros;
  std::string imu_send_topic;
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle("tmlidar_imu_adapter"));
  yamlRead<bool>(config, "send_imu_ros", send_imu_ros, false);
  yamlRead<std::string>(config["ros"], "ros_send_imu_topic", imu_send_topic, "tmlidar_imu");

  if (send_imu_ros)
  {
    imu_pub_ = nh_->advertise<sensor_msgs::Imu>(imu_send_topic, 10);
  }
}

inline void ImuAdapter::sendImu(const ImuMsg& msg)
{
  for(size_t i = 0 ; i < msg.imu_vec_ptr->size() ; i++){
    imu_pub_.publish(toRosMsg(msg,i)); 
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
class ImuAdapter : virtual public AdapterBase
{
public:
  ImuAdapter() = default;
  ~ImuAdapter() = default;
  void init(const YAML::Node& config);
  void sendImu(const ImuMsg& msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  
  double last_bag_btwn = 0;
};

inline void ImuAdapter::init(const YAML::Node& config)
{
  bool send_imu_ros;
  std::string imu_send_topic;
  node_ptr_.reset(new rclcpp::Node("tmlidar_imu_adapter"));
  yamlRead<bool>(config, "send_imu_ros", send_imu_ros, false);
  yamlRead<std::string>(config["ros"], "ros_send_imu_topic", imu_send_topic, "tmlidar_imu");
  if (send_imu_ros)
  { 
    imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(imu_send_topic, 10);
  }
}

inline void ImuAdapter::sendImu(const ImuMsg& msg)
{
  //  if(last_bag_btwn>0){
  //  printf("bag_btwn_: %f \n",  msg.imu_timestamp - last_bag_btwn);
  //  }
  //  last_bag_btwn = (*msg.imu_vec_ptr)[msg.imu_vec_ptr->size()-1].timestamp;
  
  for(size_t i = 0; i< msg.imu_vec_ptr->size();i++){
     imu_pub_->publish(toRosMsg(msg,i));
    // if(i>0){
    //   printf("bag_in_diff = %f \n",  (*msg.imu_vec_ptr)[i].timestamp - (*msg.imu_vec_ptr)[i-1].timestamp );
    // }
   
  }
}

}  // namespace lidar
}  // namespace timoo
#endif  // ROS2_FOUND