#ifndef __YHS_FW_CAN_CONTROL_NODE_H__
#define __YHS_FW_CAN_CONTROL_NODE_H__

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

#include "yhs_can_interfaces/msg/fw_io_cmd.hpp"
#include "yhs_can_interfaces/msg/fw_ctrl_cmd.hpp"
#include "yhs_can_interfaces/msg/fw_chassis_info_fb.hpp"
#include "yhs_can_interfaces/msg/fw_steering_ctrl_cmd.hpp" //dxs
#include "yhs_can_interfaces/msg/fw_steering_ctrl_fb.hpp" //dxs
#include "yhs_can_interfaces/msg/web_io_cmd.hpp"
#include "yhs_can_interfaces/msg/web_ctrl_cmd.hpp"

#include "yhs_can_control/yhs_chassis.hpp"

namespace can_control
{
  class FwCanControl : public yhs_chassis::CanControl
  {

  public:
    FwCanControl(rclcpp::Node::SharedPtr);
    ~FwCanControl();

    bool Run() override;
    void Stop() override;

  private:
    rclcpp::Node::SharedPtr node_;
    std::string if_name_;
    bool use_tf_;
    int can_socket_;
    std::thread thread_;
    std::mutex mutex_;
    double slipangle_;//dxs 
    double imu_roll_, imu_pitch_, imu_yaw_;
    bool web_unlock_{false};        // 记录小程序端解锁状态
    bool web_lamp_on_{false};       // 记录小程序端灯光状态
    unsigned char io_cmd_count_{0};   // IO 指令帧序列号（独立成员，避免 static 被多路径共享导致序列号混乱）
    unsigned char ctrl_cmd_count_{0};  // ctrl 指令帧序列号（独立成员，与 IO 序列号完全隔离）
    unsigned char steer_cmd_count_{0}; // 阿克曼转向帧序列号（独立成员）
    std::mutex can_write_mutex_;       // 保护 CAN 写操作和序列号，防止多线程竞争

    // 独立的 IO 保活定时器：以固定周期发送 IO 解锁帧，与 ctrl 帧完全解耦
    // 复刻遥控器行为：IO 帧和 ctrl 帧各自独立发送，互不干扰，避免转向抖动
    rclcpp::TimerBase::SharedPtr io_keepalive_timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<yhs_can_interfaces::msg::FwIoCmd>::SharedPtr io_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::FwCtrlCmd>::SharedPtr ctrl_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::WebIoCmd>::SharedPtr web_io_cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::WebCtrlCmd>::SharedPtr web_ctrl_cmd_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
    rclcpp::Subscription<yhs_can_interfaces::msg::FwSteeringCtrlCmd>::SharedPtr steering_ctrl_cmd_subscriber_;

    rclcpp::Publisher<yhs_can_interfaces::msg::FwChassisInfoFb>::SharedPtr chassis_info_fb_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void steering_ctrl_cmd_callback(const yhs_can_interfaces::msg::FwSteeringCtrlCmd::SharedPtr steering_ctrl_cmd_msg);// dxs add

    void FwIoCmdCallBack(const yhs_can_interfaces::msg::FwIoCmd::SharedPtr io_cmd_msg);

    void FwCtrlCmdCallBack(const yhs_can_interfaces::msg::FwCtrlCmd::SharedPtr ctrl_cmd_msg);
    void WebIoCmdCallBack(const yhs_can_interfaces::msg::WebIoCmd::SharedPtr io_cmd_msg);
    void WebCtrlCmdCallBack(const yhs_can_interfaces::msg::WebCtrlCmd::SharedPtr ctrl_cmd_msg);

    void CmdCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_msg);

    bool WaitForCanFrame();

    void CanDataRecvCallBack();

    void ImuDataCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_data_msg);

    void PublishOdom(const uint8_t gear, const double linear_vel, const double angular_vel, const double slipangle);
  };

}

#endif
