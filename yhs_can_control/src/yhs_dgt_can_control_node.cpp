#include "yhs_can_control/yhs_dgt_can_control_node.hpp"

namespace can_control
{

	DgtCanControl::DgtCanControl(rclcpp::Node::SharedPtr node)
		: node_(node), if_name_("can0"), use_tf_(false), can_socket_(-1), imu_yaw_(0.0)
	{

		node_->declare_parameter("can_name", "can0");
		node_->declare_parameter("use_tf", false);

    node_->get_parameter("can_name", if_name_);
		node_->get_parameter("use_tf", use_tf_);

		io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::DgtIoCmd>(
			"io_cmd",
			1,
			std::bind(&DgtCanControl::DgtIoCmdCallBack, this, std::placeholders::_1));

		ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::DgtCtrlCmd>(
			"ctrl_cmd",
			1,
			std::bind(&DgtCanControl::DgtCtrlCmdCallBack, this, std::placeholders::_1));

		web_io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::WebIoCmd>(
			"web_io_cmd",
			1,
			std::bind(&DgtCanControl::WebIoCmdCallBack, this, std::placeholders::_1));

		web_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::WebCtrlCmd>(
			"web_ctrl_cmd",
			1,
			std::bind(&DgtCanControl::WebCtrlCmdCallBack, this, std::placeholders::_1));

		free_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::DgtFreeCtrlCmd>(
			"free_ctrl_cmd",
			1,
			std::bind(&DgtCanControl::DgtFreeCtrlCmdCallBack, this, std::placeholders::_1));

		imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
			"imu_data",
			yhs_chassis::qos_imu,
			std::bind(&DgtCanControl::ImuDataCallBack, this, std::placeholders::_1));

		cmd_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel",
			1,
			std::bind(&DgtCanControl::CmdCallBack, this, std::placeholders::_1));

		chassis_info_fb_publisher_ = node_->create_publisher<yhs_can_interfaces::msg::DgtChassisInfoFb>("chassis_info_fb", 1);

		odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
	}

	void DgtCanControl::DgtIoCmdCallBack(const yhs_can_interfaces::msg::DgtIoCmd::SharedPtr io_cmd_msg)
	{
		yhs_can_interfaces::msg::DgtIoCmd msg = *io_cmd_msg;
		static unsigned char count = 0;
		unsigned char sendDataTemp[8] = {0};

		sendDataTemp[0] = 0xff;
		if (msg.io_cmd_lamp_ctrl)
			sendDataTemp[0] &= 0xff;
		else
			sendDataTemp[0] &= 0xfe;
		if (msg.io_cmd_unlock)
			sendDataTemp[0] &= 0xff;
		else
			sendDataTemp[0] &= 0xfd;

		sendDataTemp[1] = 0xff;
		if (msg.io_cmd_lower_beam_headlamp)
			sendDataTemp[1] &= 0xff;
		else
			sendDataTemp[1] &= 0xfe;
		if (msg.io_cmd_upper_beam_headlamp)
			sendDataTemp[1] &= 0xff;
		else
			sendDataTemp[1] &= 0xfd;

		if (msg.io_cmd_turn_lamp == 0)
			sendDataTemp[1] &= 0xf3;
		if (msg.io_cmd_turn_lamp == 1)
			sendDataTemp[1] &= 0xf7;
		if (msg.io_cmd_turn_lamp == 2)
			sendDataTemp[1] &= 0xfb;

		if (msg.io_cmd_braking_lamp)
			sendDataTemp[1] &= 0xff;
		else
			sendDataTemp[1] &= 0xef;
		if (msg.io_cmd_clearance_lamp)
			sendDataTemp[1] &= 0xff;
		else
			sendDataTemp[1] &= 0xdf;
		if (msg.io_cmd_fog_lamp)
			sendDataTemp[1] &= 0xff;
		else
			sendDataTemp[1] &= 0xbf;

		sendDataTemp[2] = msg.io_cmd_speaker;

		count++;
		if (count == 16)
			count = 0;

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

	void DgtCanControl::DgtCtrlCmdCallBack(const yhs_can_interfaces::msg::DgtCtrlCmd::SharedPtr ctrl_cmd_msg)
	{
		yhs_can_interfaces::msg::DgtCtrlCmd msg = *ctrl_cmd_msg;
		const short linear = msg.ctrl_cmd_linear * 1000;
		const short angular = msg.ctrl_cmd_angular * 100;
		const unsigned char gear = msg.ctrl_cmd_gear;

		static unsigned char count = 0;
		unsigned char sendDataTemp[8] = {0};

		sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

		sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linear & 0x0f) << 4));

		sendDataTemp[1] = (linear >> 4) & 0xff;

		sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linear >> 12));

		sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

		sendDataTemp[3] = (angular >> 4) & 0xff;

		sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

		count++;
		if (count == 16)
			count = 0;

		sendDataTemp[6] = count << 4;

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

	void DgtCanControl::WebIoCmdCallBack(const yhs_can_interfaces::msg::WebIoCmd::SharedPtr io_cmd_msg)
	{
		if(io_cmd_msg->chassis_type != yhs_can_interfaces::msg::WebIoCmd::CHASSIS_DGT) return;
		auto msg = std::make_shared<yhs_can_interfaces::msg::DgtIoCmd>();
		msg->io_cmd_lamp_ctrl = io_cmd_msg->io_cmd_lamp_ctrl;
		msg->io_cmd_unlock = io_cmd_msg->io_cmd_unlock;
		msg->io_cmd_lower_beam_headlamp = io_cmd_msg->io_cmd_lower_beam_headlamp;
		msg->io_cmd_upper_beam_headlamp = io_cmd_msg->io_cmd_upper_beam_headlamp;
		msg->io_cmd_turn_lamp = io_cmd_msg->io_cmd_turn_lamp;
		msg->io_cmd_braking_lamp = io_cmd_msg->io_cmd_braking_lamp;
		msg->io_cmd_clearance_lamp = io_cmd_msg->io_cmd_clearance_lamp;
		msg->io_cmd_fog_lamp = io_cmd_msg->io_cmd_fog_lamp;
		msg->io_cmd_speaker = io_cmd_msg->io_cmd_speaker;
		DgtIoCmdCallBack(msg);
	}

	void DgtCanControl::WebCtrlCmdCallBack(const yhs_can_interfaces::msg::WebCtrlCmd::SharedPtr ctrl_cmd_msg)
	{
		if(ctrl_cmd_msg->chassis_type != yhs_can_interfaces::msg::WebCtrlCmd::CHASSIS_DGT) return;
		auto msg = std::make_shared<yhs_can_interfaces::msg::DgtCtrlCmd>();
		msg->ctrl_cmd_linear = ctrl_cmd_msg->linear_x;
		msg->ctrl_cmd_angular = ctrl_cmd_msg->angular_z;
		msg->ctrl_cmd_gear = ctrl_cmd_msg->gear > 0 ? ctrl_cmd_msg->gear : 3;
		DgtCtrlCmdCallBack(msg);
	}

	void DgtCanControl::CmdCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_msg)
	{
		auto ctrl_cmd_msg = std::make_shared<yhs_can_interfaces::msg::DgtCtrlCmd>();
		ctrl_cmd_msg->ctrl_cmd_linear = cmd_msg->linear.x;
		ctrl_cmd_msg->ctrl_cmd_angular = cmd_msg->angular.z / 3.14 * 180;
		ctrl_cmd_msg->ctrl_cmd_gear = 3;

		DgtCtrlCmdCallBack(ctrl_cmd_msg);
	}

	void DgtCanControl::DgtFreeCtrlCmdCallBack(const yhs_can_interfaces::msg::DgtFreeCtrlCmd::SharedPtr free_ctrl_cmd_msg)
	{
		yhs_can_interfaces::msg::DgtFreeCtrlCmd msg = *free_ctrl_cmd_msg;
		const short linearl = msg.free_ctrl_cmd_velocity_l * 1000;
		const short linearr = msg.free_ctrl_cmd_velocity_r * 1000;
		const unsigned char gear = msg.ctrl_cmd_gear;
		static unsigned char count = 0;

		unsigned char sendDataTemp[8] = {0};

		sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

		sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((linearl & 0x0f) << 4));

		sendDataTemp[1] = (linearl >> 4) & 0xff;

		sendDataTemp[2] = sendDataTemp[2] | (0x0f & (linearl >> 12));

		sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((linearr & 0x0f) << 4));

		sendDataTemp[3] = (linearr >> 4) & 0xff;

		sendDataTemp[4] = sendDataTemp[4] | (0x0f & (linearr >> 12));

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
			RCLCPP_ERROR(node_->get_logger(), "Failed to send message: %s" , strerror(errno));
		}
	}

	bool DgtCanControl::WaitForCanFrame()
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
			RCLCPP_ERROR(node_->get_logger(), "Error waiting for CAN frame: %s" , std::strerror(errno));
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

	void DgtCanControl::CanDataRecvCallBack()
	{

		can_frame recv_frame;
		yhs_can_interfaces::msg::DgtChassisInfoFb chassis_info_msg;

		while (rclcpp::ok())
		{
			if (!WaitForCanFrame())
				continue;

			if (read(can_socket_, &recv_frame, sizeof(recv_frame)) >= 0)
			{
				switch (recv_frame.can_id)
				{
				case 0x98C4D1EF:
				{
					yhs_can_interfaces::msg::DgtCtrlFb msg;
					msg.ctrl_fb_target_gear = recv_frame.data[0] & 0x0f;
					msg.ctrl_fb_linear = static_cast<float>(static_cast<short>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 | (recv_frame.data[0] & 0xf0) >> 4)) / 1000;
					msg.ctrl_fb_angular = static_cast<float>(static_cast<short>(((recv_frame.data[4] & 0x0f) << 12) | (recv_frame.data[3] << 4) | ((recv_frame.data[2] & 0xf0) >> 4))) / 100.0;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.header.stamp = node_->get_clock()->now();
						chassis_info_msg.ctrl_fb = msg;
						chassis_info_fb_publisher_->publish(chassis_info_msg);
						PublishOdom(msg.ctrl_fb_linear, msg.ctrl_fb_angular / 180 * 3.14);
					}

					break;
				}

				case 0x98C4D7EF:
				{
					yhs_can_interfaces::msg::DgtLWheelFb msg;
					msg.l_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000.0f;
					msg.l_wheel_fb_pulse = static_cast<int32_t>((recv_frame.data[5] << 24) | (recv_frame.data[4] << 16) | (recv_frame.data[3] << 8) | recv_frame.data[2]);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.l_wheel_fb = msg;
					}

					break;
				}

				case 0x98C4D8EF:
				{
					yhs_can_interfaces::msg::DgtRWheelFb msg;
					msg.r_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000.0f;
					msg.r_wheel_fb_pulse = static_cast<int32_t>((recv_frame.data[5] << 24) | (recv_frame.data[4] << 16) | (recv_frame.data[3] << 8) | recv_frame.data[2]);

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.r_wheel_fb = msg;
					}

					break;
				}

				case 0x98C4DAEF:
				{
					yhs_can_interfaces::msg::DgtIoFb msg;
					msg.io_fb_lamp_ctrl = (recv_frame.data[0] & 0x01) != 0;
					msg.io_fb_unlock = (recv_frame.data[1] & 0x02) != 0;
					msg.io_fb_lower_beam_headlamp = (recv_frame.data[1] & 0x01) != 0;
					msg.io_fb_upper_beam_headlamp = (recv_frame.data[1] & 0x02) != 0;
					msg.io_fb_turn_lamp = (recv_frame.data[1] & 0xc0) >> 2;
					msg.io_fb_braking_lamp = (recv_frame.data[1] & 0x10) != 0;
					msg.io_fb_clearance_lamp = (recv_frame.data[1] & 0x20) != 0;
					msg.io_fb_fog_lamp = (recv_frame.data[1] & 0x40) != 0;
					msg.io_fb_speaker = (recv_frame.data[2] & 0x01) != 0;
					msg.io_fb_fl_impact_sensor = (recv_frame.data[3] & 0x01) != 0;
					msg.io_fb_fm_impact_sensor = (recv_frame.data[3] & 0x02) != 0;
					msg.io_fb_fr_impact_sensor = (recv_frame.data[3] & 0x04) != 0;
					msg.io_fb_rl_impact_sensor = (recv_frame.data[3] & 0x08) != 0;
					msg.io_fb_rm_impact_sensor = (recv_frame.data[3] & 0x10) != 0;
					msg.io_fb_rr_impact_sensor = (recv_frame.data[3] & 0x20) != 0;
					msg.io_fb_fl_drop_sensor = (recv_frame.data[4] & 0x01) != 0;
					msg.io_fb_fm_drop_sensor = (recv_frame.data[4] & 0x02) != 0;
					msg.io_fb_fr_drop_sensor = (recv_frame.data[4] & 0x04) != 0;
					msg.io_fb_rl_drop_sensor = (recv_frame.data[4] & 0x08) != 0;
					msg.io_fb_rm_drop_sensor = (recv_frame.data[4] & 0x10) != 0;
					msg.io_fb_rr_drop_sensor = (recv_frame.data[4] & 0x20) != 0;
					msg.io_fb_estop = (recv_frame.data[5] & 0x01) != 0;
					msg.io_fb_joypad_ctrl = (recv_frame.data[5] & 0x02) != 0;
					msg.io_fb_charge_state = (recv_frame.data[5] & 0x04) != 0;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.io_fb = msg;
					}

					break;
				}

				case 0x98C4E1EF:
				{
					yhs_can_interfaces::msg::DgtBmsFb msg;
					msg.bms_fb_voltage = static_cast<float>(reinterpret_cast<unsigned short &>(recv_frame.data[0])) / 100;
					msg.bms_fb_current = static_cast<float>(reinterpret_cast<short &>(recv_frame.data[2])) / 100;
					msg.bms_fb_remaining_capacity = static_cast<float>(reinterpret_cast<unsigned short &>(recv_frame.data[4])) / 100;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.bms_fb = msg;
					}

					break;
				}

				case 0x98C4E2EF:
				{
					yhs_can_interfaces::msg::DgtBmsFlagFb msg;
					msg.bms_flag_fb_soc = recv_frame.data[0];
					msg.bms_flag_fb_single_ov = (0x01 & recv_frame.data[1]) != 0;
					msg.bms_flag_fb_single_uv = (0x02 & recv_frame.data[1]) != 0;
					msg.bms_flag_fb_ov = (0x04 & recv_frame.data[1]) != 0;
					msg.bms_flag_fb_uv = (0x08 & recv_frame.data[1]) != 0;
					msg.bms_flag_fb_charge_ot = (0x10 & recv_frame.data[1]) != 0;
					msg.bms_flag_fb_charge_ut = (0x20 & recv_frame.data[1]) != 0;
					msg.bms_flag_fb_discharge_ot = (0x40 & recv_frame.data[1]) != 0;
					msg.bms_flag_fb_discharge_ut = (0x80 & recv_frame.data[1]) != 0;
					msg.bms_flag_fb_charge_oc = (0x01 & recv_frame.data[2]) != 0;
					msg.bms_flag_fb_discharge_oc = (0x02 & recv_frame.data[2]) != 0;
					msg.bms_flag_fb_short = (0x04 & recv_frame.data[2]) != 0;
					msg.bms_flag_fb_ic_error = (0x08 & recv_frame.data[2]) != 0;
					msg.bms_flag_fb_lock_mos = (0x10 & recv_frame.data[2]) != 0;
					msg.bms_flag_fb_charge_flag = (0x20 & recv_frame.data[2]) != 0;

					const float kTemperatureConversionFactor = 0.1;
					msg.bms_flag_fb_hight_temperature = static_cast<float>((static_cast<short>(recv_frame.data[4] << 4 | recv_frame.data[3] >> 4))) * kTemperatureConversionFactor;
					msg.bms_flag_fb_low_temperature = static_cast<float>((static_cast<short>((recv_frame.data[6] & 0x0f) << 8 | recv_frame.data[5]))) * kTemperatureConversionFactor;

					unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

					if (crc == recv_frame.data[7])
					{
						chassis_info_msg.bms_flag_fb = msg;
					}

					break;
				}

				default:
					break;
				}
			}
			//		  loop_rate.sleep();
		}
	}

	void DgtCanControl::ImuDataCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_data_msg)
	{
		std::lock_guard<std::mutex> lock(mutex_);

		tf2::Quaternion quaternion;
		tf2::fromMsg(imu_data_msg->orientation, quaternion);

		tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
	}

	void DgtCanControl::PublishOdom(const double linear_vel, const double angular_vel)
	{
		static double x_ = 0.0;
		static double y_ = 0.0;
		static double theta_ = 0.0;
		static rclcpp::Time last_time_ = node_->now();

		rclcpp::Time current_time = node_->now();

		double dt = (current_time - last_time_).seconds();

		x_ += linear_vel * cos(theta_) * dt;
		y_ += linear_vel * sin(theta_) * dt;

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
		twist_cov.twist.linear.x = linear_vel;
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

	DgtCanControl::~DgtCanControl()
	{
	}

	bool DgtCanControl::Run() 
	{
		can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (can_socket_ < 0)
		{
			RCLCPP_ERROR(node_->get_logger(), "Failed to open socket: %s" , strerror(errno));
			return false;
		}

		struct ifreq ifr;
		strncpy(ifr.ifr_name, if_name_.c_str(), IFNAMSIZ - 1);
		ifr.ifr_name[IFNAMSIZ - 1] = '\0';
		if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0)
		{
			RCLCPP_ERROR(node_->get_logger(), "Failed to get interface index: %s ==> %s" , strerror(errno) , if_name_.c_str());
			return false;
		}

		struct sockaddr_can addr;
		memset(&addr, 0, sizeof(addr));
		addr.can_family = AF_CAN;
		addr.can_ifindex = ifr.ifr_ifindex;
		if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
		{
			RCLCPP_ERROR(node_->get_logger(), "Failed to bind socket: %s" , strerror(errno));
			return false;
		}

		thread_ = std::thread(&DgtCanControl::CanDataRecvCallBack, this);

		return true;
	}

	void DgtCanControl::Stop() 
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
