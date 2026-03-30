#ifndef __YHS_DGT_CAN_CONTROL_NODE_H__
#define __YHS_DGT_CAN_CONTROL_NODE_H__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <mutex>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

#include "yhs_can_interfaces/msg/dgt_io_cmd.hpp"
#include "yhs_can_interfaces/msg/dgt_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/dgt_free_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/dgt_chassis_info_fb.hpp"
#include "yhs_can_interfaces/msg/web_io_cmd.hpp"
#include "yhs_can_interfaces/msg/web_ctrl_cmd.hpp"

#include "yhs_can_control/yhs_chassis.hpp"

#include "car_type_conf.h"

namespace can_control
{
  class DgtCanControl : public yhs_chassis::CanControl
  {

  public:
    DgtCanControl(rclcpp::Node::SharedPtr);
    ~DgtCanControl();

    bool Run() override;
    void Stop() override;

  private:
    rclcpp::Node::SharedPtr node_;
    std::string if_name_;
    bool use_tf_;
    int can_socket_;
    std::thread thread_;
    std::mutex mutex_;

    double imu_roll_, imu_pitch_, imu_yaw_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<yhs_can_interfaces::msg::DgtIoCmd>::SharedPtr io_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::DgtCtrlCmd>::SharedPtr ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::WebIoCmd>::SharedPtr web_io_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::WebCtrlCmd>::SharedPtr web_ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::DgtFreeCtrlCmd>::SharedPtr free_ctrl_cmd_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;

    rclcpp::Publisher<yhs_can_interfaces::msg::DgtChassisInfoFb>::SharedPtr chassis_info_fb_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void DgtIoCmdCallBack(const yhs_can_interfaces::msg::DgtIoCmd::SharedPtr io_cmd_msg);

    void DgtCtrlCmdCallBack(const yhs_can_interfaces::msg::DgtCtrlCmd::SharedPtr ctrl_cmd_msg);
    void WebIoCmdCallBack(const yhs_can_interfaces::msg::WebIoCmd::SharedPtr io_cmd_msg);
    void WebCtrlCmdCallBack(const yhs_can_interfaces::msg::WebCtrlCmd::SharedPtr ctrl_cmd_msg);

    void CmdCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_msg);

    void DgtFreeCtrlCmdCallBack(const yhs_can_interfaces::msg::DgtFreeCtrlCmd::SharedPtr free_ctrl_cmd_msg);

    bool WaitForCanFrame();

    void CanDataRecvCallBack();

    void ImuDataCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_data_msg);

    void PublishOdom(const double linear_vel, const double angular_vel);
  };

}

#endif
