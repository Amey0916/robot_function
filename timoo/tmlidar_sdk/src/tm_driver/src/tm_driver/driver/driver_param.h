#pragma once
#include <string>
#include <tm_driver/common/common_header.h>
namespace timoo
{
namespace lidar
{
enum LidarType  ///< LiDAR type
{
  TM16 = 1,
  TM32,
  TM64,
  TM128,
  TM300
};

enum SplitFrameMode
{
  SPLIT_BY_ANGLE = 1,
  SPLIT_BY_FIXED_PKTS,
  SPLIT_BY_CUSTOM_PKTS
};

typedef struct TMCameraTriggerParam  ///< Camera trigger parameters
{
  std::map<double, std::string> trigger_map;  ///< Map stored the trigger angle and camera frame id
  void print() const                          
  {
    TM_INFO << "------------------------------------------------------" << TM_REND;
    TM_INFO << "             Timoo Camera Trigger Parameters " << TM_REND;
    for (auto iter : trigger_map)
    {
      TM_INFOL << "camera_frame_id: " << iter.second << " trigger_angle : " << iter.first << TM_REND;
    }
    TM_INFO << "------------------------------------------------------" << TM_REND;
  }
} TMCameraTriggerParam;

typedef struct TMTransformParam  ///< The Point transform parameter
{
  float x = 0.0f;      ///< unit, m
  float y = 0.0f;      ///< unit, m
  float z = 0.0f;      ///< unit, m
  float roll = 0.0f;   ///< unit, radian
  float pitch = 0.0f;  ///< unit, radian
  float yaw = 0.0f;    ///< unit, radian
  void print() const   
  {
    TM_INFO << "------------------------------------------------------" << TM_REND;
    TM_INFO << "             Timoo Transform Parameters " << TM_REND;
    TM_INFOL << "x: " << x << TM_REND;
    TM_INFOL << "y: " << y << TM_REND;
    TM_INFOL << "z: " << z << TM_REND;
    TM_INFOL << "roll: " << roll << TM_REND;
    TM_INFOL << "pitch: " << pitch << TM_REND;
    TM_INFOL << "yaw: " << yaw << TM_REND;
    TM_INFO << "------------------------------------------------------" << TM_REND;
  }
} TMTransformParam;

typedef struct TMDecoderParam  ///< LiDAR decoder parameter
{
  float max_distance = 200.0f;                                       ///< Max distance of point cloud range
  float min_distance = 0.2f;                                         ///< Minimum distance of point cloud range
  float start_angle = 0.0f;                                          ///< Start angle of point cloud
  float end_angle = 360.0f;                                          ///< End angle of point cloud
  int left_min_angle = 0;
  int left_max_angle = 180;
  int right_min_angle = 180;
  int right_max_angle = 360;
  int cut_start_angle = 90;                                         
  int cut_end_angle = 270;  
  std::string hidelineNumStr;
  SplitFrameMode split_frame_mode = SplitFrameMode::SPLIT_BY_ANGLE;  ///< 1: Split frames by cut_angle;
                                                                     ///< 2: Split frames by fixed number of packets;
                                                                     ///< 3: Split frames by custom number of packets (num_pkts_split)
  uint32_t num_pkts_split = 1;         ///< Number of packets in one frame, only be used when split_frame_mode=3
  float cut_angle = 0.0f;              ///< Cut angle(degree) used to split frame, only be used when split_frame_mode=1
  bool use_lidar_clock = false;        ///< true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool use_imu = false;                ///< true: use imu; false: not use imu
  TMTransformParam transform_param;    ///< Used to transform points
  TMCameraTriggerParam trigger_param;  ///< Used to trigger camera
  void print() const                  
  {
    transform_param.print();
    trigger_param.print();
    TM_INFO << "------------------------------------------------------" << TM_REND;
    TM_INFO << "             Timoo Decoder Parameters " << TM_REND;
    TM_INFOL << "max_distance: " << max_distance << TM_REND;
    TM_INFOL << "min_distance: " << min_distance << TM_REND;
    // TM_INFOL << "start_angle: " << start_angle << TM_REND;
    // TM_INFOL << "end_angle: " << end_angle << TM_REND;
    TM_INFOL << "hidelineNumStr: " << hidelineNumStr << TM_REND;
    TM_INFOL << "left_min_angle: " << left_min_angle << TM_REND;
    TM_INFOL << "left_max_angle: " << left_max_angle << TM_REND;
    TM_INFOL << "right_min_angle: " << right_min_angle << TM_REND;
    TM_INFOL << "right_max_angle: " << right_max_angle << TM_REND;

    TM_INFOL << "use_lidar_clock: " << use_lidar_clock << TM_REND;
    TM_INFOL << "split_frame_mode: " << split_frame_mode << TM_REND;
    TM_INFOL << "num_pkts_split: " << num_pkts_split << TM_REND;
    TM_INFOL << "cut_angle: " << cut_angle << TM_REND;
    TM_INFO << "------------------------------------------------------" << TM_REND;
  }
} TMDecoderParam;

typedef struct TMInputParam  ///< The LiDAR input parameter
{
  std::string device_ip = "192.168.1.201";     ///< Ip of LiDAR
  std::string multi_cast_address = "0.0.0.0";  ///< Address of multicast
  uint16_t msop_port = 2368;                   ///< Msop packet port number
  uint16_t difop_port = 8603;                  ///< Difop packet port number
  bool read_pcap = false;          ///< true: The driver will process the pcap through pcap_path. false: The driver will
                                   ///< Get data from online LiDAR
  double pcap_rate = 1;            ///< Rate to read the pcap file
  bool pcap_repeat = true;         ///< true: The pcap bag will repeat play
  std::string pcap_path = "null";  ///< Absolute path of pcap file
  void print() const             
  {
    TM_INFO << "------------------------------------------------------" << TM_REND;
    TM_INFO << "             Timoo Input Parameters " << TM_REND;
    TM_INFOL << "multi_cast_address: " << multi_cast_address << TM_REND;
    TM_INFOL << "msop_port: " << msop_port << TM_REND;
    TM_INFOL << "difop_port: " << difop_port << TM_REND;
    TM_INFOL << "read_pcap: " << read_pcap << TM_REND;
    TM_INFOL << "pcap_repeat: " << pcap_repeat << TM_REND;
    TM_INFOL << "pcap_path: " << pcap_path << TM_REND;
    TM_INFO << "------------------------------------------------------" << TM_REND;
  }
} TMInputParam;

typedef struct TMDriverParam  ///< The LiDAR driver parameter
{
  TMInputParam input_param;                ///< Input parameter
  TMDecoderParam decoder_param;            ///< Decoder parameter
  // std::string angle_path = "null";         ///< Path of angle calibration files(angle.csv).Only used for internal debugging.
  std::string frame_id = "tmlidar";        ///< The frame id of LiDAR message
  LidarType lidar_type = LidarType::TM16;  ///< Lidar type
  bool wait_for_difop = true;              ///< true: start sending point cloud until receive difop packet
  bool saved_by_rows = false;  ///< true: the output point cloud will be saved by rows (default is saved by columns)
  void print() const           
  {
    input_param.print();
    decoder_param.print();
    TM_INFO << "------------------------------------------------------" << TM_REND;
    TM_INFOL << "             Timoo Driver Parameters " << TM_REND;
    // TM_INFOL << "angle_path: " << angle_path << TM_REND;
    TM_INFOL << "frame_id: " << frame_id << TM_REND;
    TM_INFOL << "lidar_type: ";
    TM_INFO << lidarTypeToStr(lidar_type) << TM_REND;
    TM_INFOL << "------------------------------------------------------" << TM_REND;
  }
  static std::string lidarTypeToStr(const LidarType& type)
  {
    std::string str = "";
    switch (type)
    {
      case LidarType::TM16:
        str = "TM16";
        break;
      case LidarType::TM32:
        str = "TM32";
        break;
      case LidarType::TM64:
        str = "TM64";
        break;
      case LidarType::TM128:
        str = "TM128";
        break;
      case LidarType::TM300:
        str = "TM300";
        break;
      default:
        str = "ERROR";
        TM_ERROR << "TM_ERROR" << TM_REND;
    }
    return str;
  }
  static LidarType strToLidarType(const std::string& type)
  {
    if (type == "TM16")
    {
      return lidar::LidarType::TM16;
    }
    else if (type == "TM32")
    {
      return lidar::LidarType::TM32;
    }
    else if (type == "TM64")
    {
      return lidar::LidarType::TM64;
    }
    else if (type == "TM128")
    {
      return lidar::LidarType::TM128;
    }
    else if (type == "TM300")
    {
      return lidar::LidarType::TM300;
    }
    else
    {
      TM_ERROR << "Wrong lidar type: " << type << TM_REND;
      TM_ERROR << "Please setup the correct type: TM16, TM32, TM64, TM128, TM300" << TM_REND;
      exit(-1);
    }
  }
} TMDriverParam;
}  // namespace lidar
}  // namespace timoo
