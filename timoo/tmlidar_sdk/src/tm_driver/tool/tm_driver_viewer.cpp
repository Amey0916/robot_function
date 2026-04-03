#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "tm_driver/api/lidar_driver.h"
using namespace timoo::lidar;
using namespace pcl::visualization;
std::shared_ptr<PCLVisualizer> pcl_viewer;
std::mutex mex_viewer;

bool checkKeywordExist(int argc, const char* const* argv, const char* str)
{
  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      return true;
    }
  }
  return false;
}

bool parseArgument(int argc, const char* const* argv, const char* str, std::string& val)
{
  int index = -1;
  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      index = i + 1;
    }
  }
  if (index > 0 && index < argc)
  {
    val = argv[index];
    return true;
  }
  return false;
}

void parseParam(int argc, char* argv[], TMDriverParam& param)
{
  param.wait_for_difop = false;
  std::string result_str;
  if (parseArgument(argc, argv, "-type", result_str))
  {
    param.lidar_type = param.strToLidarType(result_str);
  }
  if (parseArgument(argc, argv, "-msop", result_str))
  {
    param.input_param.msop_port = std::stoi(result_str);
  }
  if (parseArgument(argc, argv, "-difop", result_str))
  {
    param.input_param.difop_port = std::stoi(result_str);
  }
  if (parseArgument(argc, argv, "-x", result_str))
  {
    param.decoder_param.transform_param.x = std::stof(result_str);
  }
  if (parseArgument(argc, argv, "-y", result_str))
  {
    param.decoder_param.transform_param.y = std::stof(result_str);
  }
  if (parseArgument(argc, argv, "-z", result_str))
  {
    param.decoder_param.transform_param.z = std::stof(result_str);
  }
  if (parseArgument(argc, argv, "-roll", result_str))
  {
    param.decoder_param.transform_param.roll = std::stof(result_str);
  }
  if (parseArgument(argc, argv, "-pitch", result_str))
  {
    param.decoder_param.transform_param.pitch = std::stof(result_str);
  }
  if (parseArgument(argc, argv, "-yaw", result_str))
  {
    param.decoder_param.transform_param.yaw = std::stof(result_str);
  }
  if (parseArgument(argc, argv, "-pcap", param.input_param.pcap_path))
  {
    param.input_param.read_pcap = true;
  }
}

void printHelpMenu()
{
  TM_MSG << "Arguments are: " << TM_REND;
  TM_MSG << "        -msop             = LiDAR msop port number,the default value is 2368" << TM_REND;
  TM_MSG << "        -difop            = LiDAR difop port number,the default value is 8603" << TM_REND;
  TM_MSG << "        -type             = LiDAR type( TM16, TM32, TM64, TM128, TM300), the default "
            "value is TM16"
         << TM_REND;
  TM_MSG << "        -x                = Transformation parameter, unit: m " << TM_REND;
  TM_MSG << "        -y                = Transformation parameter, unit: m " << TM_REND;
  TM_MSG << "        -z                = Transformation parameter, unit: m " << TM_REND;
  TM_MSG << "        -roll             = Transformation parameter, unit: radian " << TM_REND;
  TM_MSG << "        -pitch            = Transformation parameter, unit: radian " << TM_REND;
  TM_MSG << "        -yaw              = Transformation parameter, unit: radian " << TM_REND;
  TM_MSG << "        -pcap             = The path of the pcap file, if this argument is set, the driver "
            "will work in off-line mode and read the pcap file. Otherwise the driver work in online mode."
         << TM_REND;
}

void printParam(const TMDriverParam& param)
{
  if (param.input_param.read_pcap)
  {
    TM_INFOL << "Working mode: ";
    TM_INFO << "Offline Pcap " << TM_REND;
    TM_INFOL << "Pcap Path: ";
    TM_INFO << param.input_param.pcap_path << TM_REND;
  }
  else
  {
    TM_INFOL << "Working mode: ";
    TM_INFO << "Online LiDAR " << TM_REND;
  }
  TM_INFOL << "MSOP Port: ";
  TM_INFO << param.input_param.msop_port << TM_REND;
  TM_INFOL << "DIFOP Port: ";
  TM_INFO << param.input_param.difop_port << TM_REND;
  TM_INFOL << "LiDAR Type: ";
  TM_INFO << param.lidarTypeToStr(param.lidar_type) << TM_REND;
  TM_INFOL << "Transformation Parameters (x, y, z, roll, pitch, yaw): " << TM_REND;
  TM_INFOL << "x: ";
  TM_INFO << std::fixed << param.decoder_param.transform_param.x << TM_REND;
  TM_INFOL << "y: ";
  TM_INFO << std::fixed << param.decoder_param.transform_param.y << TM_REND;
  TM_INFOL << "z: ";
  TM_INFO << std::fixed << param.decoder_param.transform_param.z << TM_REND;
  TM_INFOL << "roll: ";
  TM_INFO << std::fixed << param.decoder_param.transform_param.roll << TM_REND;
  TM_INFOL << "pitch: ";
  TM_INFO << std::fixed << param.decoder_param.transform_param.pitch << TM_REND;
  TM_INFOL << "yaw: ";
  TM_INFO << std::fixed << param.decoder_param.transform_param.yaw << TM_REND;
}
/**
 * @brief The point cloud callback function. This function will be registered to lidar driver.
 *              When the point cloud message is ready, driver can send out messages through this function.
 * @param msg  The lidar point cloud message.
 */
void pointCloudCallback(const PointCloudMsg<pcl::PointXYZI>& msg)
{
  /* Note: Please do not put time-consuming operations in the callback function! */
  /* Make a copy of the message and process it in another thread is recommended*/
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_pointcloud->points.assign(msg.point_cloud_ptr->begin(), msg.point_cloud_ptr->end());
  pcl_pointcloud->height = msg.height;
  pcl_pointcloud->width = msg.width;
  pcl_pointcloud->is_dense = false;
  PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pcl_pointcloud, "intensity");
  {
    const std::lock_guard<std::mutex> lock(mex_viewer);
    pcl_viewer->updatePointCloud<pcl::PointXYZI>(pcl_pointcloud, point_color_handle, "tmlidar");
  }
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
  TM_TITLE << "            TM_Driver Viewer Version: v" << TMLIDAR_VERSION_MAJOR << "." << TMLIDAR_VERSION_MINOR << "."
           << TMLIDAR_VERSION_PATCH << TM_REND;
  TM_TITLE << "------------------------------------------------------" << TM_REND;

  if (argc < 2)
  {
    TM_INFOL << "Use 'tm_driver_viewer -h/--help' to check the argument menu..." << TM_REND;
  }
  if (checkKeywordExist(argc, argv, "-h") || checkKeywordExist(argc, argv, "--help"))
  {
    printHelpMenu();
    return 0;
  }
  pcl_viewer = std::make_shared<PCLVisualizer>("TMPointCloudViewer");

  pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
  pcl_viewer->addCoordinateSystem(1.0);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_viewer->addPointCloud<pcl::PointXYZI>(pcl_pointcloud, "tmlidar");
  pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "tmlidar");

  LidarDriver<pcl::PointXYZI> driver;  ///< Declare the driver object
  TMDriverParam param;                 ///< Create a parameter object
  parseParam(argc, argv, param);
  printParam(param);
  driver.regExceptionCallback(exceptionCallback);  ///< Register the exception callback function into the driver
  driver.regRecvCallback(pointCloudCallback);      ///< Register the point cloud callback function into the driver
  if (!driver.init(param))                         ///< Call the init function and pass the parameter
  {
    TM_ERROR << "Driver Initialize Error..." << TM_REND;
    return -1;
  }
  driver.start();  ///< The driver thread will start
  TM_INFO << "Timoo Lidar-Driver Viewer start......" << TM_REND;

  while (!pcl_viewer->wasStopped())
  {
    {
      const std::lock_guard<std::mutex> lock(mex_viewer);
      pcl_viewer->spinOnce();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
