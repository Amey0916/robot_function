#include "yhs_can_control/yhs_fr_can_control_node.hpp"

namespace can_control
{

	FrCanControl::FrCanControl(rclcpp::Node::SharedPtr node)
		: node_(node), if_name_("can0"), use_tf_(false), can_socket_(-1), imu_yaw_(0.0),wheel_base_(0.6)
	{

		node_->declare_parameter("can_name", "can0");
		node_->declare_parameter("use_tf", false);
		node_->declare_parameter("wheel_base", 0.6);

    node_->get_parameter("can_name", if_name_);
		node_->get_parameter("use_tf", use_tf_);
		node_->get_parameter("wheel_base", wheel_base_);

		io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::FrIoCmd>(
			"io_cmd",
			1,
			std::bind(&FrCanControl::FrIoCmdCallBack, this, std::placeholders::_1));

		ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::FrCtrlCmd>(
			"ctrl_cmd",
			1,
			std::bind(&FrCanControl::FrCtrlCmdCallBack, this, std::placeholders::_1));

		web_io_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::WebIoCmd>(
			"web_io_cmd",
			1,
			std::bind(&FrCanControl::WebIoCmdCallBack, this, std::placeholders::_1));

		web_ctrl_cmd_subscriber_ = node_->create_subscription<yhs_can_interfaces::msg::WebCtrlCmd>(
			"web_ctrl_cmd",
			1,
			std::bind(&FrCanControl::WebCtrlCmdCallBack, this, std::placeholders::_1));

		imu_subscriber_ = node_->create_subscription<sensor_msgs::msg::Imu>(
			"imu_data",
			yhs_chassis::qos_imu,
			std::bind(&FrCanControl::ImuDataCallBack, this, std::placeholders::_1));

		cmd_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
			"cmd_vel",
			1,
			std::bind(&FrCanControl::CmdCallBack, this, std::placeholders::_1));

		chassis_info_fb_publisher_ = node_->create_publisher<yhs_can_interfaces::msg::FrChassisInfoFb>("chassis_info_fb", 1);

		odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
	}

	void FrCanControl::FrIoCmdCallBack(const yhs_can_interfaces::msg::FrIoCmd::SharedPtr io_cmd_msg)
	{
		yhs_can_interfaces::msg::FrIoCmd msg = *io_cmd_msg;
		static unsigned char count = 0;
		unsigned char sendDataTemp[8] = {0};

		sendDataTemp[0] = msg.io_cmd_enable;

		if(msg.io_cmd_upper_beam_headlamp)
			sendDataTemp[1] |= 0x20;

		if(msg.io_cmd_turn_lamp == 0)
			sendDataTemp[1] |= 0x00;
		if(msg.io_cmd_turn_lamp == 1)
			sendDataTemp[1] |= 0x04;
		if(msg.io_cmd_turn_lamp == 2)
			sendDataTemp[1] |= 0x08;

		sendDataTemp[2] = msg.io_cmd_speaker;

		count ++;
		if(count == 16)	count = 0;

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

	void FrCanControl::FrCtrlCmdCallBack(const yhs_can_interfaces::msg::FrCtrlCmd::SharedPtr ctrl_cmd_msg)
	{
		yhs_can_interfaces::msg::FrCtrlCmd msg = *ctrl_cmd_msg;
		const unsigned short vel = msg.ctrl_cmd_velocity * 1000;
		const short angular = msg.ctrl_cmd_steering * 100;
		const unsigned char gear = msg.ctrl_cmd_gear;
		const unsigned char brake = msg.ctrl_cmd_brake;

		static unsigned char count = 0;
		unsigned char sendDataTemp[8] = {0};

		if(msg.ctrl_cmd_velocity < 0) return;

		sendDataTemp[0] = sendDataTemp[0] | (0x0f & gear);

		sendDataTemp[0] = sendDataTemp[0] | (0xf0 & ((vel & 0x0f) << 4));

		sendDataTemp[1] = (vel >> 4) & 0xff;

		sendDataTemp[2] = sendDataTemp[2] | (0x0f & (vel >> 12));

		sendDataTemp[2] = sendDataTemp[2] | (0xf0 & ((angular & 0x0f) << 4));

		sendDataTemp[3] = (angular >> 4) & 0xff;

		sendDataTemp[4] = sendDataTemp[4] | (0xf0 & ((brake & 0x0f) << 4));

		sendDataTemp[4] = sendDataTemp[4] | (0x0f & (angular >> 12));

		sendDataTemp[5] = (brake >> 4) & 0x0f;

		count ++;

		if(count == 16)	count = 0;

		sendDataTemp[6] =  count << 4;

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

	void FrCanControl::WebIoCmdCallBack(const yhs_can_interfaces::msg::WebIoCmd::SharedPtr io_cmd_msg)
	{
		if(io_cmd_msg->chassis_type != yhs_can_interfaces::msg::WebIoCmd::CHASSIS_FR) return;
		auto msg = std::make_shared<yhs_can_interfaces::msg::FrIoCmd>();
		msg->io_cmd_enable = io_cmd_msg->io_cmd_enable;
		msg->io_cmd_upper_beam_headlamp = io_cmd_msg->io_cmd_upper_beam_headlamp;
		msg->io_cmd_turn_lamp = io_cmd_msg->io_cmd_turn_lamp;
		msg->io_cmd_speaker = io_cmd_msg->io_cmd_speaker;
		FrIoCmdCallBack(msg);
	}

	void FrCanControl::WebCtrlCmdCallBack(const yhs_can_interfaces::msg::WebCtrlCmd::SharedPtr ctrl_cmd_msg)
	{
		constexpr uint8_t kForwardGear = 4;
		constexpr uint8_t kReverseGear = 2;
		if(ctrl_cmd_msg->chassis_type != yhs_can_interfaces::msg::WebCtrlCmd::CHASSIS_FR) return;
		auto msg = std::make_shared<yhs_can_interfaces::msg::FrCtrlCmd>();
		msg->ctrl_cmd_velocity = std::fabs(ctrl_cmd_msg->velocity) > 1e-6 ? ctrl_cmd_msg->velocity : std::fabs(ctrl_cmd_msg->linear_x);
		msg->ctrl_cmd_steering = ctrl_cmd_msg->steering;
		msg->ctrl_cmd_brake = ctrl_cmd_msg->brake;
		msg->ctrl_cmd_gear = ctrl_cmd_msg->gear > 0 ? ctrl_cmd_msg->gear : (ctrl_cmd_msg->linear_x > 0 ? kForwardGear : kReverseGear);
		FrCtrlCmdCallBack(msg);
	}

	void FrCanControl::CmdCallBack(const geometry_msgs::msg::Twist::SharedPtr cmd_msg)
	{
		auto ctrl_cmd_msg = std::make_shared<yhs_can_interfaces::msg::FrCtrlCmd>();

		ctrl_cmd_msg->ctrl_cmd_velocity = fabs(cmd_msg->linear.x);
		ctrl_cmd_msg->ctrl_cmd_steering = cmd_msg->angular.z / 3.14 * 180;
		ctrl_cmd_msg->ctrl_cmd_gear = cmd_msg->linear.x > 0 ? 4 : 2;
		ctrl_cmd_msg->ctrl_cmd_brake = 0.0;

		FrCtrlCmdCallBack(ctrl_cmd_msg);
	}

	bool FrCanControl::WaitForCanFrame()
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

	void FrCanControl::CanDataRecvCallBack()
	{

		can_frame recv_frame;
		yhs_can_interfaces::msg::FrChassisInfoFb chassis_info_msg;

		while (rclcpp::ok())
		{
			if(!WaitForCanFrame()) continue;
			
			if(read(can_socket_, &recv_frame, sizeof(recv_frame)) >= 0)
			{
				switch (recv_frame.can_id)
				{
					//
					case 0x98C4D2EF:
					{
						yhs_can_interfaces::msg::FrCtrlFb msg;
					
						msg.ctrl_fb_gear = 0x0f & recv_frame.data[0];
					
						msg.ctrl_fb_velocity = static_cast<float>(static_cast<unsigned int>((recv_frame.data[2] & 0x0f) << 12 | recv_frame.data[1] << 4 | (recv_frame.data[0] & 0xf0) >> 4)) / 1000;
					
						msg.ctrl_fb_steering = static_cast<float>(static_cast<short>((recv_frame.data[4] & 0x0f) << 12 | recv_frame.data[3] << 4 | (recv_frame.data[2] & 0xf0) >> 4)) / 100;

						msg.ctrl_fb_brake = (recv_frame.data[4] & 0xf0) >> 4 | (recv_frame.data[5] & 0x0f) << 4;
					
						msg.ctrl_fb_mode = (recv_frame.data[5] & 0x30) >> 4;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){	
							chassis_info_msg.header.stamp = node_->get_clock()->now();
							chassis_info_msg.ctrl_fb = msg;
							chassis_info_fb_publisher_->publish(chassis_info_msg);
							if(msg.ctrl_fb_gear == 2) msg.ctrl_fb_velocity = -msg.ctrl_fb_velocity;
							PublishOdom(msg.ctrl_fb_velocity, msg.ctrl_fb_steering/180*3.1415);
						}

						break;
					}

					//
					case 0x98C4D7EF:
					{
						yhs_can_interfaces::msg::FrLrWheelFb msg;
					
						msg.lr_wheel_fb_velocity = static_cast<float>(static_cast<float>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

						msg.lr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.lr_wheel_fb = msg;	
						}

						break;
					}

					//
					case 0x98C4D8EF:
					{
						yhs_can_interfaces::msg::FrRrWheelFb msg;
					
						msg.rr_wheel_fb_velocity = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 1000;

						msg.rr_wheel_fb_pulse = static_cast<int>(recv_frame.data[5] << 24 | recv_frame.data[4] << 16 | recv_frame.data[3] << 8 | recv_frame.data[2]);

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.rr_wheel_fb = msg;	
						}

						break;
					}

					//
					case 0x98C4DAEF:
					{
						yhs_can_interfaces::msg::FrIoFb msg;
					
						msg.io_fb_enable = (recv_frame.data[0] & 0x01) != 0;
						msg.io_fb_upper_beam_headlamp = (recv_frame.data[1] & 0x02) != 0;

						msg.io_fb_turn_lamp = (0x0c & recv_frame.data[1]) >> 2;

						msg.io_fb_braking_lamp = (0x10 & recv_frame.data[1]) != 0;
						msg.io_fb_speaker = (0x01 & recv_frame.data[2]) != 0;
						msg.io_fb_fm_impact_sensor = (0x02 & recv_frame.data[3]) != 0;
						msg.io_fb_rm_impact_sensor = (0x10 & recv_frame.data[3]) != 0;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.io_fb = msg;	
						}

						break;
					}


					//
					case 0x98C4E1EF:
					{
						yhs_can_interfaces::msg::FrBmsInfoFb msg;
					
						msg.bms_info_voltage = static_cast<float>(static_cast<short>(recv_frame.data[1] << 8 | recv_frame.data[0])) / 100;

						msg.bms_info_current = static_cast<float>(static_cast<short>(recv_frame.data[3] << 8 | recv_frame.data[2])) / 100;

						msg.bms_info_remaining_capacity = static_cast<float>(static_cast<unsigned short>(recv_frame.data[5] << 8 | recv_frame.data[4])) / 100;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.bms_info_fb = msg;	
						}

						break;
					}

					//
					case 0x98C4E2EF:
					{
						yhs_can_interfaces::msg::FrBmsFlagInfoFb msg;
					
						msg.bms_flag_info_soc = recv_frame.data[0];

						msg.bms_flag_info_single_ov = (recv_frame.data[1] & 0x01) != 0;
						msg.bms_flag_info_single_uv = (recv_frame.data[1] & 0x02) != 0;
						msg.bms_flag_info_ov = (recv_frame.data[1] & 0x04) != 0;
						msg.bms_flag_info_uv = (recv_frame.data[1] & 0x08) != 0;
						msg.bms_flag_info_charge_ot = (recv_frame.data[1] & 0x10) != 0;
						msg.bms_flag_info_charge_ut = (recv_frame.data[1] & 0x20) != 0;
						msg.bms_flag_info_discharge_ot = (recv_frame.data[1] & 0x40) != 0;
						msg.bms_flag_info_discharge_ut = (recv_frame.data[1] & 0x80) != 0;

						msg.bms_flag_info_charge_oc = (recv_frame.data[2] & 0x01) != 0;
						msg.bms_flag_info_discharge_oc = (recv_frame.data[2] & 0x02) != 0;
						msg.bms_flag_info_short = (recv_frame.data[2] & 0x04) != 0;
						msg.bms_flag_info_ic_error = (recv_frame.data[2] & 0x08) != 0;
						msg.bms_flag_info_lock_mos = (recv_frame.data[2] & 0x10) != 0;
						msg.bms_flag_info_charge_flag = (recv_frame.data[2] & 0x20) != 0;


						msg.bms_flag_info_hight_temperature = static_cast<float>(static_cast<short>(recv_frame.data[4] << 4 | recv_frame.data[3] >> 4)) / 10;
						msg.bms_flag_info_low_temperature = static_cast<float>(static_cast<short>((recv_frame.data[6] & 0x0f) << 8 | recv_frame.data[5])) / 10;

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.bms_flag_info_fb = msg;	
						}

						break;
					}

					//
					case 0x98C4DCEF:
					{
						yhs_can_interfaces::msg::FrDriveMcuEcodeFb msg;
					
						msg.drive_fb_mcuecode = static_cast<int>(recv_frame.data[3] << 24 | recv_frame.data[2] << 16 | recv_frame.data[1] << 8 | recv_frame.data[0]); 

						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.drive_mcu_ecode_fb = msg;	
						}

						break;
					}

					//
					case 0x98C4EAEF:
					{
						yhs_can_interfaces::msg::FrVehDiagFb msg;
					
						msg.veh_fb_fault_level = 0x0f & recv_frame.data[0];

						msg.veh_fb_auto_can_ctrl_cmd = (recv_frame.data[0] & 0x10) != 0;
						msg.veh_fb_auto_io_can_cmd = (recv_frame.data[0] & 0x20) != 0;
						msg.veh_fb_eps_dis_on_line = (recv_frame.data[1] & 0x01) != 0;
						msg.veh_fb_eps_fault = (recv_frame.data[1] & 0x02) != 0;
						msg.veh_fb_eps_mosf_et_ot = (recv_frame.data[1] & 0x04) != 0;
						msg.veh_fb_eps_warning = (recv_frame.data[1] & 0x08) != 0;
						msg.veh_fb_eps_dis_work = (recv_frame.data[1] & 0x10) != 0;
						msg.veh_fb_eps_over_current = (recv_frame.data[1] & 0x20) != 0;
						msg.veh_fb_ehb_ecu_fault = (recv_frame.data[2] & 0x10) != 0;
						msg.veh_fb_ehb_dis_on_line = (recv_frame.data[2] & 0x20) != 0;
						msg.veh_fb_ehb_work_model_fault = (recv_frame.data[2] & 0x40) != 0;
						msg.veh_fb_ehb_dis_en = (recv_frame.data[2] & 0x80) != 0;
						msg.veh_fb_ehb_anguler_fault = (recv_frame.data[3] & 0x01) != 0;
						msg.veh_fb_ehb_ot = (recv_frame.data[3] & 0x02) != 0;
						msg.veh_fb_ehb_power_fault = (recv_frame.data[3] & 0x04) != 0;
						msg.veh_fb_ehb_sensor_abnomal = (recv_frame.data[3] & 0x08) != 0;
						msg.veh_fb_ehb_motor_fault = (recv_frame.data[3] & 0x10) != 0;
						msg.veh_fb_ehb_oil_press_sensor_fault = (recv_frame.data[3] & 0x20) != 0;
						msg.veh_fb_ehb_oil_fault = (recv_frame.data[3] & 0x40) != 0;
						msg.veh_fb_drv_mcu_dis_on_line = (recv_frame.data[4] & 0x01) != 0;
						msg.veh_fb_drv_mcu_ot = (recv_frame.data[4] & 0x02) != 0;
						msg.veh_fb_drv_mcu_ov = (recv_frame.data[4] & 0x04) != 0;
						msg.veh_fb_drv_mcu_uv = (recv_frame.data[4] & 0x08) != 0;
						msg.veh_fb_drv_mcu_short = (recv_frame.data[4] & 0x10) != 0;
						msg.veh_fb_drv_mcu_scram = (recv_frame.data[4] & 0x20) != 0;
						msg.veh_fb_drv_mcu_hall = (recv_frame.data[4] & 0x40) != 0;
						msg.veh_fb_drv_mcu_mosf_ef = (recv_frame.data[4] & 0x80) != 0;
						msg.veh_fb_aux_bms_dis_on_line = (recv_frame.data[5] & 0x10) != 0;
						msg.veh_fb_aux_scram = recv_frame.data[5] & 0x20;
						msg.veh_fb_aux_remote_close = recv_frame.data[5] & 0x40;
						msg.veh_fb_aux_remote_dis_on_line = recv_frame.data[5] & 0x80;


						unsigned char crc = recv_frame.data[0] ^ recv_frame.data[1] ^ recv_frame.data[2] ^ recv_frame.data[3] ^ recv_frame.data[4] ^ recv_frame.data[5] ^ recv_frame.data[6];

						if(crc == recv_frame.data[7]){
							chassis_info_msg.veh_diag_fb = msg;
						}


						break;
					}

					default:
						break;
				}			
			}
		}
	}

	void FrCanControl::ImuDataCallBack(const sensor_msgs::msg::Imu::SharedPtr imu_data_msg)
	{
		std::lock_guard<std::mutex> lock(mutex_);

		tf2::Quaternion quaternion;
		tf2::fromMsg(imu_data_msg->orientation, quaternion);

		tf2::Matrix3x3(quaternion).getRPY(imu_roll_, imu_pitch_, imu_yaw_);
	}

	void FrCanControl::PublishOdom(const double velocity, const double steering)
	{
		static double x_ = 0.0;
		static double y_ = 0.0;
		static double theta_ = 0.0;
		static rclcpp::Time last_time_ = node_->now();

		rclcpp::Time current_time = node_->now();

		double dt = (current_time - last_time_).seconds();

    double linear_vel = velocity;
		double angular_vel = linear_vel * tan(steering) / wheel_base_;

    double x_mid = 0.0;
    double y_mid = 0.0;

		x_ += linear_vel * cos(theta_) * dt;
		y_ += linear_vel * sin(theta_) * dt;

    x_mid  = x_ + wheel_base_ / 2 * cos(theta_);
    y_mid  = y_ + wheel_base_ / 2 * sin(theta_);

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
		pose_cov.pose.position.x = x_mid;
		pose_cov.pose.position.y = y_mid;
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

	FrCanControl::~FrCanControl()
	{
	}

	bool FrCanControl::Run() 
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

		thread_ = std::thread(&FrCanControl::CanDataRecvCallBack, this);

		return true;
	}

	void FrCanControl::Stop() 
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
