#include <iostream>
#include <string>
#include <sensor_msgs/msg/imu.hpp>
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#ifdef __cplusplus
extern "C"{
#endif

#include "ch_serial.h"

#define IMU_SERIAL  ("/dev/imu")
#define BAUD        (B115200)
#define GRA_ACC     (9.8)
#define DEG_TO_RAD  (0.01745329)
#define BUF_SIZE    (1024)
#ifdef __cplusplus
}
#endif

using namespace std::chrono_literals;
using namespace std;
static raw_t raw;

rmw_qos_profile_t qos_profile_imu{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  2000,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

auto qos_imu = rclcpp::QoS(
rclcpp::QoSInitialization(
    qos_profile_imu.history,
    qos_profile_imu.depth
),
qos_profile_imu);

class IMUPublisher : public rclcpp::Node
{
	public:
		int fd = 0;
		uint8_t buf[BUF_SIZE] = {0};
		
		std::vector<std::string> cmd_list;

		IMUPublisher() : Node("IMU_publisher")	
		{
			fd = open_serial();  
			
			cmd_list.push_back("AT+SETPTL=A0,B0,D0,D1\r\n");
		  cmd_list.push_back("AT+URFR=1,0,0,0,1,0,0,0,1\r\n");
		  cmd_list.push_back("AT+RST\r\n");
      for (const std::string& str : cmd_list) 
      {
        std::vector<uint8_t> data = stringToAscii(str);
        write(fd, data.data(), data.size());
      }

			imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", qos_imu);
      imu_angle_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("rpy", 10);
			timer_ = this->create_wall_timer(10ms, std::bind(&IMUPublisher::timer_callback, this));
		}

	private: 
	  std::vector<uint8_t> stringToAscii(const std::string& str)
	  {
      std::vector<uint8_t> asciiVec;
      for (char c : str) 
      {
        asciiVec.push_back(static_cast<uint8_t>(c));
      }
      return asciiVec;
    }

		void timer_callback()
		{
			auto imu_data = sensor_msgs::msg::Imu();
			int n = read(fd, buf, sizeof(buf));

			for(int i = 0; i < n; i++)
			{
				int rev = ch_serial_input(&raw, buf[i]);
				
				if(raw.item_code[raw.nitem_code - 1] != KItemGWSOL)
				{
					if(rev)
					{
						imu_data.orientation.w = raw.imu[raw.nimu - 1].quat[0];
						imu_data.orientation.x = raw.imu[raw.nimu - 1].quat[1];	
						imu_data.orientation.y = raw.imu[raw.nimu - 1].quat[2];
						imu_data.orientation.z = raw.imu[raw.nimu - 1].quat[3];
						imu_data.angular_velocity.x = raw.imu[raw.nimu - 1].gyr[0] * DEG_TO_RAD;
						imu_data.angular_velocity.y = raw.imu[raw.nimu - 1].gyr[1] * DEG_TO_RAD;
						imu_data.angular_velocity.z = raw.imu[raw.nimu - 1].gyr[2] * DEG_TO_RAD;
						imu_data.linear_acceleration.x = raw.imu[raw.nimu - 1].acc[0] * GRA_ACC;
						imu_data.linear_acceleration.y = raw.imu[raw.nimu - 1].acc[1] * GRA_ACC;
						imu_data.linear_acceleration.z = raw.imu[raw.nimu - 1].acc[2] * GRA_ACC;

						imu_data.header.stamp = rclcpp::Clock().now();
						imu_data.header.frame_id = "imu_link";
						if(imu_data.orientation.w != 0)
							imu_pub->publish(imu_data);

              tf2::Quaternion quat;
              tf2::fromMsg(imu_data.orientation, quat);

              double roll, pitch, yaw;
              tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

              auto angles_msg = std::make_shared<geometry_msgs::msg::Vector3>();
              angles_msg->x = roll * 180.0 / M_PI;
              angles_msg->y = pitch * 180.0 / M_PI;
              angles_msg->z = yaw * 180.0 / M_PI;

              imu_angle_pub_->publish(*angles_msg);
					}
				}
			}

			memset(buf,0,sizeof(buf));
		}

		int open_serial(void)
		{
			struct termios options;

			int fd = open(IMU_SERIAL, O_RDWR | O_NOCTTY);
			tcgetattr(fd,&options);

			if(fd == -1)
			{
				perror("unable to open serial port");
				exit(0);
			}
			
			if(fcntl(fd, F_SETFL, 0) < 0)
				cout << "fcntl failed" << "\n" << endl;
			else
				fcntl(fd, F_SETFL, 0);

			if(isatty(STDIN_FILENO) == 0)
				cout << "isatty success!" << "\n" << endl;
			else
				cout << "isatty success!" << "\n" << endl;

			memset(&options, 0, sizeof(options));

			options.c_cflag = BAUD | CS8 | CLOCAL | CREAD;
			options.c_iflag = IGNPAR;
			options.c_oflag = 0;
			options.c_lflag = 0;
			options.c_cc[VTIME] = 0;
			options.c_cc[VMIN] = 0;
			tcflush(fd, TCIFLUSH);
			tcsetattr(fd, TCSANOW, &options);

			return fd;
		}

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr imu_angle_pub_;
};


int main(int argc, const char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<IMUPublisher>());
	rclcpp::shutdown();

	return 0;
}
