#include "tm_driver/api/lidar_driver.h"
using namespace timoo::lidar;

struct PointXYZI  ///< user defined point type
{
  float x;
  float y;
  float z;
  uint8_t intensity;
};

/**
 * @brief The point cloud callback function. This function will be registered to lidar driver.
 *              When the point cloud message is ready, driver can send out messages through this function.
 * @param msg  The lidar point cloud message.
 */
void pointCloudCallback(const PointCloudMsg<PointXYZI>& msg)
{
  /* Note: Please do not put time-consuming operations in the callback function! */
  /* Make a copy of the message and process it in another thread is recommended*/
  TM_MSG << "msg: " << msg.seq << " point cloud size: " << msg.point_cloud_ptr->size() << TM_REND;
}

/**
 * @brief The exception callback function. This function will be registered to lidar driver.
 * @param code The error code struct.
 */
void exceptionCallback(const Error& code)
{
  /* Note: Please do not put time-consuming operations in the callback function! */
  /* Make a copy of the error message and process it in another thread is recommended*/
  TM_WARNING << code.toString() << TM_REND;
}

int main(int argc, char* argv[])
{
  TM_TITLE << "------------------------------------------------------" << TM_REND;
  TM_TITLE << "            TM_Driver Core Version: v" << TMLIDAR_VERSION_MAJOR << "." << TMLIDAR_VERSION_MINOR << "."
           << TMLIDAR_VERSION_PATCH << TM_REND;
  TM_TITLE << "------------------------------------------------------" << TM_REND;

  LidarDriver<PointXYZI> driver;  ///< Declare the driver object

  TMDriverParam param;                                         ///< Create a parameter object
  param.input_param.read_pcap = true;                          ///< Set read_pcap to true
  param.input_param.pcap_path = "/home/timoo/lidar.pcap";  ///< Set the pcap file directory
  param.input_param.msop_port = 2368;                          ///< Set the lidar msop port number, the default is 2368
  param.input_param.difop_port = 8603;                         ///< Set the lidar difop port number, the default is 8603
  param.lidar_type = LidarType::TM16;                          ///< Set the lidar type. Make sure this type is correct
  param.print();

  driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function into the driver
  driver.regRecvCallback(pointCloudCallback);      ///< Register the point cloud callback function into the driver
  if (!driver.init(param))                         ///< Call the init function and pass the parameter
  {
    TM_ERROR << "Driver Initialize Error..." << TM_REND;
    return -1;
  }
  driver.start();  ///< The driver thread will start
  TM_DEBUG << "Timoo Lidar-Driver Linux pcap demo start......" << TM_REND;

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}