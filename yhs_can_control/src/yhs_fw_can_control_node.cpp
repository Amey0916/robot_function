#include "yhs_can_control/yhs_fw_can_control_node.hpp"

namespace can_control
{

  FwCanControl::FwCanControl(rclcpp::Node::SharedPtr node)
      : node_(node), if_name_("can0"), use_tf_(false), can_socket_(-1), imu_yaw_(0.0)
  {

    node_->declare_parameter("can_name", "can0");
    node_->declare_parameter("use_tf", false);

    node_->get_parameter("can_name", if_name_);
    node_->get_parameter("use_tf", use_tf_);

    io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::FwIoCmd>(
        "io_cmd",
        1,
        std::bind(&FwCanControl::FwIoCmdCallBack, this, std::placeholders::_1));

    ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::FwCtrlCmd>(
        "ctrl_cmd",
        1,
        std::bind(&FwCanControl::FwCtrlCmdCallBack, this, std::placeholders::_1));

    web_io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::WebIoCmd>(
        "web_io_cmd",
        1,
        std::bind(&FwCanControl::WebIoCmdCallBack, this, std::placeholders::_1));

    web_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::WebCtrlCmd>(
        "web_ctrl_cmd",
        1,
        std::bind(&FwCanControl::WebCtrlCmdCallBack, this, std::placeholders::_1));

    imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data",
        yhs_chassis::qos_imu,
        std::bind(&FwCanControl::ImuDataCallBack, this, std::placeholders::_1));

    cmd_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        1,
        std::bind(&FwCanControl::CmdCallBack, this, std::placeholders::_1));

    steering_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::FwSteeringCtrlCmd>(
        "steering_ctrl_cmd",
        1,
        std::bind(&FwCanControl::steering_ctrl_cmd_callback, this, std::placeholders::_1));

    chassis_info_fb_publisher_ = node_->create_publisher<yhs_can_interfaces::msg::FwChassisInfoFb>("chassis_info_fb", 1);

    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    // 独立 IO 保活定时器（20ms 周期，每秒50次）
    // 频率足够高，底盘 MCU 的超时回锁机制永远不会触发
    // 遥控器路径（FwIoCmdCallBack/io_cmd topic）完全不受影响
    io_keepalive_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {
        if (!web_unlock_) return;
        auto io_msg = std::make_shared<yhs_can_interfaces::msg::FwIoCmd>();
        io_msg->io_cmd_lamp_ctrl           = true;
        io_msg->io_cmd_unlock              = web_unlock_;
        io_msg->io_cmd_lower_beam_headlamp = web_lamp_on_;
        io_msg->io_cmd_upper_beam_headlamp = false;
        io_msg->io_cmd_turn_lamp           = 0;
        io_msg->io_cmd_braking_lamp        = false;
        io_msg->io_cmd_clearance_lamp      = false;
        io_msg->io_cmd_fog_lamp            = false;
        io_msg->io_cmd_speaker             = 0;
        io_msg->io_cmd_wireless_charge     = 0;
        FwIoCmdCallBack(io_msg);
      });

  }

  void FwCanControl::FwIoCmdCallBack(const yhs_can_interfaces::msg::FwIoCmd::SharedPtr io_cmd_msg)
  {
    yhs_can_interfaces::msg::FwIoCmd msg = *io_cmd_msg;
    // 使用成员变量而非 static，避免 io_keepalive_timer_ 和 WebIoCmdCallBack
    // 两条路径同时调用时共享序列号导致 count 不连续，底盘 MCU 触发安全复位（抖动根因）
    unsigned char count = io_cmd_count_;
    io_cmd_count_ = (io_cmd_count_ + 1) % 16;

    unsigned char sendDataTemp[8] = {0};

    if (msg.io_cmd_lamp_ctrl)
      sendDataTemp[0] |= 0x01;
    if (msg.io_cmd_unlock)
      sendDataTemp[0] |= 0x02;

    if (msg.io_cmd_lower_beam_headlamp)
      sendDataTemp[1] |= 0x01;
    if (msg.io_cmd_upper_beam_headlamp)
      sendDataTemp[1] |= 0x02;

    if (msg.io_cmd_turn_lamp == 0)
      sendDataTemp[1] |= 0x00;
    if (msg.io_cmd_turn_lamp == 1)
      sendDataTemp[1] |= 0x04;
    if (msg.io_cmd_turn_lamp == 2)
      sendDataTemp[1] |= 0x08;

    if (msg.io_cmd_braking_lamp)
      sendDataTemp[1] |= 0x10;
    if (msg.io_cmd_clearance_lamp)
      sendDataTemp[1] |= 0x20;
    if (msg.io_cmd_fog_lamp)
      sendDataTemp[1] |= 0x40;

    sendDataTemp[2] = msg.io_cmd_speaker;

    sendDataTemp[3] = msg.io_cmd_wireless_charge;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D7D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send message: %s" , strerror(errno));
    }
  }

  void FwCanControl::FwCtrlCmdCallBack(const yhs_can_interfaces::msg::FwCtrlCmd::SharedPtr ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::FwCtrlCmd msg = *ctrl_cmd_msg;
    const short ctrl_cmd_x_linear = msg.ctrl_cmd_x_linear * 1000;
    const short ctrl_cmd_z_angular = msg.ctrl_cmd_z_angular * 100;
    const short ctrl_cmd_y_linear = msg.ctrl_cmd_y_linear * 1000;
    const unsigned char gear = msg.ctrl_cmd_gear;

    // ctrl 指令帧序列号使用成员变量，与 IO 帧序列号完全隔离
    unsigned char count = ctrl_cmd_count_;
    ctrl_cmd_count_ = (ctrl_cmd_count_ + 1) % 16;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((ctrl_cmd_x_linear & 0x0f) << 4));

    sendDataTemp[1] = (ctrl_cmd_x_linear >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (ctrl_cmd_x_linear >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((ctrl_cmd_z_angular & 0x0f) << 4));

    sendDataTemp[3] = (ctrl_cmd_z_angular >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (ctrl_cmd_z_angular >> 12));

    sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((ctrl_cmd_y_linear & 0x0f) << 4));

    sendDataTemp[5] = (ctrl_cmd_y_linear >> 4) & 0xff;

    sendDataTemp[6] = sendDataTemp[6] | (0x0f & (ctrl_cmd_y_linear >> 12));

    sendDataTemp[6] = sendDataTemp[6] | (count << 4);

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D1D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send message: %s" , strerror(errno));
    }
  }

  void FwCanControl::WebIoCmdCallBack(const yhs_can_interfaces::msg::WebIoCmd::SharedPtr io_cmd_msg)
  {
    if(io_cmd_msg->chassis_type != yhs_can_interfaces::msg::WebIoCmd::CHASSIS_FW) return;
    // 记录小程序端的解锁/灯光状态，供 io_keepalive_timer_ 独立定时发送保活帧使用
    web_unlock_ = io_cmd_msg->io_cmd_unlock;
    web_lamp_on_ = io_cmd_msg->io_cmd_lower_beam_headlamp;
    auto msg = std::make_shared<yhs_can_interfaces::msg::FwIoCmd>();
    msg->io_cmd_lamp_ctrl = true;  // 始终置1，确保底盘处理此IO帧
    msg->io_cmd_unlock = io_cmd_msg->io_cmd_unlock;
    msg->io_cmd_lower_beam_headlamp = io_cmd_msg->io_cmd_lower_beam_headlamp;
    msg->io_cmd_upper_beam_headlamp = io_cmd_msg->io_cmd_upper_beam_headlamp;
    msg->io_cmd_turn_lamp = io_cmd_msg->io_cmd_turn_lamp;
    msg->io_cmd_braking_lamp = io_cmd_msg->io_cmd_braking_lamp;
    msg->io_cmd_clearance_lamp = io_cmd_msg->io_cmd_clearance_lamp;
    msg->io_cmd_fog_lamp = io_cmd_msg->io_cmd_fog_lamp;
    msg->io_cmd_speaker = io_cmd_msg->io_cmd_speaker;
    msg->io_cmd_wireless_charge = io_cmd_msg->io_cmd_wireless_charge;
    FwIoCmdCallBack(msg);
  }

  void FwCanControl::WebCtrlCmdCallBack(const yhs_can_interfaces::msg::WebCtrlCmd::SharedPtr ctrl_cmd_msg)
  {
    constexpr uint8_t kFrontAckermannGear = 4;
    constexpr uint8_t kFrontDifferentialGear = 6;
    constexpr uint8_t kFrontOmniGear = 7;
    if(ctrl_cmd_msg->chassis_type != yhs_can_interfaces::msg::WebCtrlCmd::CHASSIS_FW) return;

    // IO 解锁保活由 io_keepalive_timer_ 独立定时发送（50ms周期），
    // 与 ctrl 帧完全解耦，避免每次运动指令都触发底盘解锁状态复位导致转向抖动

    // gear=6(差速) 或 gear=7(全向) 时，直接走 FwCtrlCmd
    const uint8_t gear = ctrl_cmd_msg->gear;
    if(gear == kFrontDifferentialGear || gear == kFrontOmniGear)
    {
      auto msg = std::make_shared<yhs_can_interfaces::msg::FwCtrlCmd>();
      msg->ctrl_cmd_x_linear = ctrl_cmd_msg->linear_x;
      msg->ctrl_cmd_y_linear = ctrl_cmd_msg->linear_y;
      msg->ctrl_cmd_z_angular = ctrl_cmd_msg->angular_z;
      msg->ctrl_cmd_gear = gear;
      FwCtrlCmdCallBack(msg);
      return;
    }

    // gear=4(阿克曼) 或未指定 gear 时，沿用原有逻辑
    if(std::fabs(ctrl_cmd_msg->linear_y) > 1e-6 || std::fabs(ctrl_cmd_msg->angular_z) > 1e-6)
    {
      auto msg = std::make_shared<yhs_can_interfaces::msg::FwCtrlCmd>();
      msg->ctrl_cmd_x_linear = ctrl_cmd_msg->linear_x;
      msg->ctrl_cmd_y_linear = ctrl_cmd_msg->linear_y;
      msg->ctrl_cmd_z_angular = ctrl_cmd_msg->angular_z;
      msg->ctrl_cmd_gear = gear > 0 ? gear : (std::fabs(ctrl_cmd_msg->linear_y) > 1e-6 ? kFrontOmniGear : kFrontDifferentialGear);
      FwCtrlCmdCallBack(msg);
      return;
    }

    // 阿克曼转向控制（velocity + steering）
    auto msg = std::make_shared<yhs_can_interfaces::msg::FwSteeringCtrlCmd>();
    msg->steering_ctrl_cmd_velocity = std::fabs(ctrl_cmd_msg->velocity) > 1e-6 ? ctrl_cmd_msg->velocity : std::fabs(ctrl_cmd_msg->linear_x);
    msg->steering_ctrl_cmd_steering = ctrl_cmd_msg->steering;
    msg->ctrl_cmd_gear = gear > 0 ? gear : kFrontAckermannGear;
    steering_ctrl_cmd_callback(msg);
  }

// dxs add D2D0
  void FwCanControl::steering_ctrl_cmd_callback(const yhs_can_interfaces::msg::FwSteeringCtrlCmd::SharedPtr steering_ctrl_cmd_msg)
  {
    yhs_can_interfaces::msg::FwSteeringCtrlCmd msg = *steering_ctrl_cmd_msg;
    const short steering_ctrl_cmd_velocity = msg.steering_ctrl_cmd_velocity * 1000;
    const short steering_ctrl_cmd_steering = msg.steering_ctrl_cmd_steering * 100;
    const unsigned char gear = msg.ctrl_cmd_gear;

    static unsigned char count = 0;
    unsigned char sendDataTemp[8] = {0};

    sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

    sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((steering_ctrl_cmd_velocity & 0x0f) << 4));

    sendDataTemp[1] = (steering_ctrl_cmd_velocity >> 4) & 0xff;

    sendDataTemp[2] = sendDataTemp[2] | (0x0f & (steering_ctrl_cmd_velocity >> 12));

    sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((steering_ctrl_cmd_steering & 0x0f) << 4));

    sendDataTemp[3] = (steering_ctrl_cmd_steering >> 4) & 0xff;

    sendDataTemp[4] = sendDataTemp[4] | (0x0f & (steering_ctrl_cmd_steering >> 12));

    count++;
    if (count == 16)
      count = 0;

    sendDataTemp[6] = count << 4;

    sendDataTemp[7] = sendDataTemp[0] ^ sendDataTemp[1] ^ sendDataTemp[2] ^ sendDataTemp[3] ^ sendDataTemp[4] ^ sendDataTemp[5] ^ sendDataTemp[6];

    can_frame send_frame;

    send_frame.can_id = 0x98C4D2D0;
    send_frame.can_dlc = 8;

    memcpy(send_frame.data, sendDataTemp, 8);

    int ret = write(can_socket_, &send_frame, sizeof(send_frame));
    if (ret <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("yhs_can_control_node"), "Failed to send message: " << strerror(errno));
    }
  }

  void FwCanControl::CmdCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_msg)
  {
    auto ctrl_cmd_msg = std::make_shared<yhs_can_interfaces::msg::FwCtrlCmd>();

    if(cmd_msg->linear.y == 0.0)
    {
      ctrl_cmd_msg->ctrl_cmd_x_linear = cmd_msg->linear.x;
      ctrl_cmd_msg->ctrl_cmd_z_angular = cmd_msg->angular.z / 3.14 * 180;
      ctrl_cmd_msg->ctrl_cmd_gear = 6;
    }
    else
    {
      ctrl_cmd_msg->ctrl_cmd_x_linear = cmd_msg->linear.x;
      ctrl_cmd_msg->ctrl_cmd_y_linear = cmd_msg->linear.y;
      ctrl_cmd_msg->ctrl_cmd_gear = 7;
    }

    FwCtrlCmdCallBack(ctrl_cmd_msg);
  }

  bool FwCanControl::WaitForCanFrame()
  {
    struct timeval tv;
    fd_set rdfs;
    FD_ZERO(&rdfs);
    FD_SET(can_socket_, &rdfs);
    tv.tv_sec = 0;
    tv.tv_usec = 10000; // 10ms

    int ret = select(can_socket_ + 1, &rdfs, NULL, NULL, &tv);
    if (ret == -1)
    {
      RCLCPP_ERROR(node_->get_logger(), "Error waiting for CAN frame: %s", std::strerror(errno));
      return false;
    }
    else if (ret == 0)
    {
      RCLCPP_WARN(node_->get_logger(), "Timeout waiting for CAN frame! Please check whether the can0 setting is correct,\
whether the can line is connected correctly, and whether the chassis is powered on.");
      return false;
    }
    else
    {
      return true;
    }
    return false;
  }

  void FwCanControl::CanDataRecvCallBack()
  {
		can_frame recv_frame;
		yhs_can_interfaces::msg::FwChassisInfoFb chassis_info_msg;
	
		while (rclcpp::ok())
		{
			if(!WaitForCanFrame()) continue;
			
			if(read(can_socket_, &recv_frame, sizeof(recv_frame)) >= 0)
			{
				switch (recv_frame.can_id)
				{
					case 0x98C4D1EF:
					{
						yhs_can_interfaces::msg::FwCtrlFb msg;
						msg.ctrl_fb_gear = 0x0f & recv_frame.data[0];
					
						msg.ctrl_fb_x_linear = static_cast<float>(static_cast<short>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 | (recv_frame.data[0] & 0xf0) >> 4)) / 1000;
					
						msg.ctrl_fb_z_angular = static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 | (recv_frame.data[2] & 0xf0) >> 4)) / 100;

						msg.ctrl_fb_y_linear = static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 12 | recv_frame.data[5] << 4 | (recv_frame.data[4] & 0xf0) >> 4)) / 100;
					

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.header.stamp = node_->get_clock()->now();
							chassis_info_msg.ctrl_fb = msg;
							chassis_info_fb_publisher_->publish(chassis_info_msg);
							PublishOdom(msg.ctrl_fb_gear, msg.ctrl_fb_x_linear, msg.ctrl_fb_z_angular / 180 * 3.14, msg.ctrl_fb_y_linear);
						}

						break;
					}

          // dxs add D2EF
          case 0x98C4D2EF:
          {
            yhs_can_interfaces::msg::FwSteeringCtrlFb msg;
            msg.steering_ctrl_fb_gear = 0x0f & recv_frame.data[0];

            msg.steering_ctrl_fb_velocity = static_cast<float>(static_cast<short>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 | (recv_frame.data[0] & 0xf0) >> 4)) / 1000;

            msg.steering_ctrl_fb_steering = static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 | (recv_frame.data[2] & 0xf0) >> 4)) / 100;

            unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

            if (crc == recv_frame.data[7])
            {
              slipangle_ = msg.steering_ctrl_fb_steering;// dxs
              chassis_info_msg.steering_ctrl_fb = msg;
            }

            break;
          }
          
					case 0x98C4D6EF:
					{
						yhs_can_interfaces::msg::FwLfWheelFb msg;
						msg.lf_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

						msg.lf_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.lf_wheel_fb = msg;
						}

						break;
					}

					case 0x98C4D7EF:
					{
						yhs_can_interfaces::msg::FwLrWheelFb msg;
						msg.lr_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

						msg.lr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.lr_wheel_fb = msg;
						}

						break;
					}
					
					case 0x98C4D8EF:
					{
						yhs_can_interfaces::msg::FwRrWheelFb msg;
						msg.rr_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

						msg.rr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.rr_wheel_fb = msg;
						}

						break;
					}

					case 0x98C4D9EF:
					{
						yhs_can_interfaces::msg::FwRfWheelFb msg;
						msg.rf_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

						msg.rf_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.rf_wheel_fb = msg;
						}

						break;
					}

					case 0x98C4DAEF:
					{
						yhs_can_interfaces::msg::FwIoFb msg;
						msg.io_fb_lamp_ctrl = (recv_frame.data[0] & 0x01) != 0;
						msg.io_fb_unlock = (recv_frame.data[0] & 0x02) != 0;
						msg.io_fb_lower_beam_headlamp = (recv_frame.data[1] & 0x01) != 0;
						msg.io_fb_upper_beam_headlamp = (recv_frame.data[1] & 0x02) != 0;

						msg.io_fb_turn_lamp = (0x0c & recv_frame.data[1]) >> 2;

						msg.io_fb_braking_lamp = (0x10 & recv_frame.data[1]) != 0;
						msg.io_fb_clearance_lamp = (0x20 & recv_frame.data[1]) != 0;
						msg.io_fb_fog_lamp = (0x40 & recv_frame.data[1]) != 0;
						msg.io_fb_speaker = (0x01 & recv_frame.data[2]) != 0;
						msg.io_fb_fl_impact_sensor = (0x01 & recv_frame.data[3]) != 0;
						msg.io_fb_fm_impact_sensor = (0x02 & recv_frame.data[3]) != 0;
						msg.io_fb_fr_impact_sensor = (0x04 & recv_frame.data[3]) != 0;
						msg.io_fb_rl_impact_sensor = (0x08 & recv_frame.data[3]) != 0;
						msg.io_fb_rm_impact_sensor = (0x10 & recv_frame.data[3]) != 0;
						msg.io_fb_rr_impact_sensor = (0x20 & recv_frame.data[3]) != 0;
						msg.io_fb_fl_drop_sensor = (0x01 & recv_frame.data[4]) != 0;
						msg.io_fb_fm_drop_sensor = (0x02 & recv_frame.data[4]) != 0;
						msg.io_fb_fr_drop_sensor = (0x04 & recv_frame.data[4]) != 0;
						msg.io_fb_rl_drop_sensor = (0x08 & recv_frame.data[4]) != 0;
						msg.io_fb_rm_drop_sensor = (0x10 & recv_frame.data[4]) != 0;
						msg.io_fb_rr_drop_sensor = (0x20 & recv_frame.data[4]) != 0;
						msg.io_fb_estop = (0x01 & recv_frame.data[5]) != 0;
						msg.io_fb_joypad_ctrl = (0x02 & recv_frame.data[5]) != 0;
						msg.io_fb_charge_state = (0x04 & recv_frame.data[5]) != 0;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.io_fb = msg;
						}

						break;
					}

					case 0x98C4DCEF:
					{
						yhs_can_interfaces::msg::FwFrontAngleFb msg;
						msg.front_angle_fb_l = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

						msg.front_angle_fb_r = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.front_angle_fb = msg;
						}

						break;
					}

					case 0x98C4DDEF:
					{
						yhs_can_interfaces::msg::FwRearAngleFb msg;
						msg.rear_angle_fb_l = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

						msg.rear_angle_fb_r = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.rear_angle_fb = msg;
						}

						break;
					}

					case 0x98C4E1EF:
					{
						yhs_can_interfaces::msg::FwBmsFb msg;
						msg.bms_fb_voltage = static_cast<float>(static_cast<unsigned short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

						msg.bms_fb_current = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

						msg.bms_fb_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frame.data[5] << 8 | recv_frame.data[4])) / 100;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.bms_fb = msg;
						}

						break;
					}

					case 0x98C4E2EF:
					{
						yhs_can_interfaces::msg::FwBmsFlagFb msg;
						msg.bms_flag_fb_soc = recv_frame.data[0];

						msg.bms_flag_fb_single_ov = (recv_frame.data[1] & 0x01) != 0;
						msg.bms_flag_fb_single_uv = (recv_frame.data[1] & 0x02) != 0;
						msg.bms_flag_fb_ov = (recv_frame.data[1] & 0x04) != 0;
						msg.bms_flag_fb_uv = (recv_frame.data[1] & 0x08) != 0;
						msg.bms_flag_fb_charge_ot = (recv_frame.data[1] & 0x10) != 0;
						msg.bms_flag_fb_charge_ut = (recv_frame.data[1] & 0x20) != 0;
						msg.bms_flag_fb_discharge_ot = (recv_frame.data[1] & 0x40) != 0;
						msg.bms_flag_fb_discharge_ut = (recv_frame.data[1] & 0x80) != 0;

						msg.bms_flag_fb_charge_oc = (recv_frame.data[2] & 0x01) != 0;
						msg.bms_flag_fb_discharge_oc = (recv_frame.data[2] & 0x02) != 0;
						msg.bms_flag_fb_short = (recv_frame.data[2] & 0x04) != 0;
						msg.bms_flag_fb_ic_error = (recv_frame.data[2] & 0x08) != 0;
						msg.bms_flag_fb_lock_mos = (recv_frame.data[2] & 0x10) != 0;
						msg.bms_flag_fb_charge_flag = (recv_frame.data[2] & 0x20) != 0;

						msg.bms_flag_fb_hight_temperature = static_cast<float>(static_cast<short>(recv_frame.data[4] << 4 | recv_frame.data[3] >> 4)) / 10;

						msg.bms_flag_fb_low_temperature = static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 8 | recv_frame.data[5])) / 10;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.bms_flag_fb = msg;
						}

						break;
					}

					default:
						break;
				}			
			}
		}
	}

  void FwCanControl::ImuDataCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_data_msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    tf2::Quaternion quaternion;
    tf2::fromMsg(imu_data_msg->orientation, quaternion);

    tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
  }

  void FwCanControl::PublishOdom(const uint8_t gear, const double linear_x, const double angular_vel, const double linear_y)
  {
    static double x_ = 0.0;
    static double y_ = 0.0;
    static double theta_ = 0.0;

    static rclcpp::Time last_time_ = node_->now();

    rclcpp::Time current_time = node_->now();

    double dt = (current_time - last_time_).seconds();

    if(gear == 6)
    {
      x_ += linear_x * cos(theta_) * dt;
      y_ += linear_x * sin(theta_) * dt;

      auto publisher_count = node_->count_publishers("/imu_data");

      if (publisher_count == 0)
      {
        theta_ += angular_vel * dt;
      }
      else
      {
        std::lock_guard<std::mutex> lock(mutex_);
        theta_ = imu_yaw_;
        RCLCPP_INFO_ONCE(node_->get_logger(), "Get imu_data");
      }
    }

    else if(gear == 7)
    {
      double theta = theta_ + slipangle_;

      if(linear_x >= 0)
      {
        x_ += linear_x * cos(theta) * dt;
        y_ += linear_x * sin(theta) * dt;
      }
      else
      {
        theta = theta_ + slipangle_ + M_PI;
        x_ += -linear_x * cos(theta) * dt;
        y_ += -linear_x * sin(theta) * dt;
      }

      auto publisher_count = node_->count_publishers("/imu_data");

      if (publisher_count == 0)
      {
      }
      else
      {
        std::lock_guard<std::mutex> lock(mutex_);
        theta_ = imu_yaw_;
        RCLCPP_INFO_ONCE(node_->get_logger(), "Get imu_data");
      }     
    }



    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    geometry_msgs::msg::PoseWithCovariance pose_cov;
    pose_cov.pose.position.x = x_;
    pose_cov.pose.position.y = y_;
    pose_cov.pose.position.z = 0.0;
    tf2::Quaternion quat;
    quat.setRPY(imu_roll_, imu_pitch_, theta_);
    pose_cov.pose.orientation.x = quat.x();
    pose_cov.pose.orientation.y = quat.y();
    pose_cov.pose.orientation.z = quat.z();
    pose_cov.pose.orientation.w = quat.w();
    odom_msg.pose = pose_cov;

    geometry_msgs::msg::TwistWithCovariance twist_cov;
    twist_cov.twist.linear.x = linear_x;
    twist_cov.twist.linear.y = 0.0;
    twist_cov.twist.linear.z = 0.0;
    twist_cov.twist.angular.x = 0.0;
    twist_cov.twist.angular.y = 0.0;
    twist_cov.twist.angular.z = angular_vel;
    odom_msg.twist = twist_cov;

    // tf
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = node_->now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg.pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

    if (use_tf_)
      tf_broadcaster_->sendTransform(odom_tf);

    odom_pub_->publish(odom_msg);

    last_time_ = current_time;
  }

  FwCanControl::~FwCanControl()
  {
  }

  bool FwCanControl::Run()
  {
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to open socket: %s", strerror(errno));
      return false;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get interface index: %s ==> %s", strerror(errno), if_name_.c_str());
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to bind socket: %s", strerror(errno));
      return false;
    }

    thread_ = std::thread(&FwCanControl::CanDataRecvCallBack, this);

    return true;
  }

  void FwCanControl::Stop()
  {
    if (can_socket_ >= 0)
    {
      close(can_socket_);
      can_socket_ = -1;
    }

    if (thread_.joinable())
    {
      thread_.join();
    }
  }
}