#pragma once
#include "adapter/adapter_base.hpp"
#include "tm_driver/api/lidar_driver.h"

namespace timoo
{
namespace lidar
{
class DriverAdapter : virtual public AdapterBase
{
public:
  DriverAdapter();
  ~DriverAdapter();
  void init(const YAML::Node& config);
  void start();
  void stop();
  inline void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback);
  inline void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  inline void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  inline void regRecvCallback(const std::function<void(const ImuMsg&)>& callback);
  inline void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback);
  void decodeScan(const ScanMsg& msg);
  void decodePacket(const PacketMsg& msg);

private:
  void localPointsCallback(const PointCloudMsg<PointT>& msg);
  void localScanCallback(const ScanMsg& msg);
  void localPacketCallback(const PacketMsg& msg);
  void localImuCallback(const ImuMsg& msg);
  void localCameraTriggerCallback(const CameraTrigger& msg);
  void localExceptionCallback(const lidar::Error& msg);
  LidarPointCloudMsg core2SDK(const lidar::PointCloudMsg<PointT>& msg);

private:
  std::shared_ptr<lidar::LidarDriver<PointT>> driver_ptr_;
  std::vector<std::function<void(const LidarPointCloudMsg&)>> point_cloud_cb_vec_;
  std::vector<std::function<void(const ScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> packet_cb_vec_;
  std::vector<std::function<void(const ImuMsg&)>> imu_cb_vec_;
  std::vector<std::function<void(const CameraTrigger&)>> camera_trigger_cb_vec_;
  lidar::ThreadPool::Ptr thread_pool_ptr_;
};

inline DriverAdapter::DriverAdapter()
{
  driver_ptr_.reset(new lidar::LidarDriver<PointT>());
  thread_pool_ptr_.reset(new lidar::ThreadPool());
  driver_ptr_->regExceptionCallback(std::bind(&DriverAdapter::localExceptionCallback, this, std::placeholders::_1));
}

inline DriverAdapter::~DriverAdapter()
{
  driver_ptr_->stop();
}

inline void DriverAdapter::init(const YAML::Node& config)
{
  lidar::TMDriverParam driver_param;
  int msg_source;
  std::string lidar_type;
  uint16_t split_frame_mode;
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  yamlReadAbort<int>(config, "msg_source", msg_source);
  yamlReadAbort<bool>(config, "send_imu_ros", driver_param.decoder_param.use_imu);  // IMU
  yamlRead<std::string>(driver_config, "frame_id", driver_param.frame_id, "tmlidar");
  // yamlRead<std::string>(driver_config, "angle_path", driver_param.angle_path, "");
  yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
  yamlRead<bool>(driver_config, "wait_for_difop", driver_param.wait_for_difop, true);
  yamlRead<bool>(driver_config, "saved_by_rows", driver_param.saved_by_rows, false);
  yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.decoder_param.use_lidar_clock, false);
  yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
  yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
  yamlRead<int>(driver_config, "cut_start_angle", driver_param.decoder_param.cut_start_angle, 90);
  yamlRead<int>(driver_config, "cut_end_angle", driver_param.decoder_param.cut_end_angle, 270);
  // yamlRead<float>(driver_config, "start_angle", driver_param.decoder_param.start_angle, 0);
  // yamlRead<float>(driver_config, "end_angle", driver_param.decoder_param.end_angle, 360);
  
  yamlRead<std::string>(driver_config, "hide_line", driver_param.decoder_param.hidelineNumStr, "");
  yamlRead<int>(driver_config, "left_min_angle", driver_param.decoder_param.left_min_angle, 0);
  yamlRead<int>(driver_config, "left_max_angle", driver_param.decoder_param.left_max_angle, 180);
  yamlRead<int>(driver_config, "right_min_angle", driver_param.decoder_param.right_min_angle, 180);
  yamlRead<int>(driver_config, "right_max_angle", driver_param.decoder_param.right_max_angle, 360);
  // printf("driver_param.decoder_param.left_min_angle:%d \n",driver_param.decoder_param.left_min_angle);
  // printf("driver_param.decoder_param.left_min_angle:%d \n",driver_param.decoder_param.left_max_angle);
  // printf("driver_param.decoder_param.left_min_angle:%d \n",driver_param.decoder_param.right_min_angle);
  // printf("driver_param.decoder_param.left_min_angle:%d \n",driver_param.decoder_param.right_max_angle);

  yamlRead<uint16_t>(driver_config, "split_frame_mode", split_frame_mode, 2);
  yamlRead<uint32_t>(driver_config, "num_pkts_split", driver_param.decoder_param.num_pkts_split, 0);
  yamlRead<float>(driver_config, "cut_angle", driver_param.decoder_param.cut_angle, 0);
  yamlRead<std::string>(driver_config, "device_ip", driver_param.input_param.device_ip, "192.168.1.201");
  yamlRead<std::string>(driver_config, "multi_cast_address", driver_param.input_param.multi_cast_address, "0.0.0.0");
  yamlRead<uint16_t>(driver_config, "msop_port", driver_param.input_param.msop_port, 2368);
  yamlRead<uint16_t>(driver_config, "difop_port", driver_param.input_param.difop_port, 8603);
  yamlRead<bool>(driver_config, "read_pcap", driver_param.input_param.read_pcap, false);
  yamlRead<double>(driver_config, "pcap_rate", driver_param.input_param.pcap_rate, 1);
  yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, false);
  yamlRead<std::string>(driver_config, "pcap_path", driver_param.input_param.pcap_path, "");
  yamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
  yamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
  yamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
  yamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
  yamlRead<float>(driver_config, "pitch", driver_param.decoder_param.transform_param.pitch, 0);
  yamlRead<float>(driver_config, "yaw", driver_param.decoder_param.transform_param.yaw, 0);
  driver_param.lidar_type = driver_param.strToLidarType(lidar_type);
  driver_param.decoder_param.split_frame_mode = SplitFrameMode(split_frame_mode);
  
  if (msg_source == MsgSource::MSG_FROM_LIDAR || msg_source == MsgSource::MSG_FROM_PCAP)
  {
    if (!driver_ptr_->init(driver_param))  //diffrence between init() and initDecoderOnly()
    {
      TM_ERROR << "Driver Initialize Error...." << TM_REND;
      exit(-1);
    }
  }
  else
  {
    driver_ptr_->initDecoderOnly(driver_param);
  }
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localPointsCallback, this, std::placeholders::_1));
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localImuCallback, this, std::placeholders::_1));
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localScanCallback, this, std::placeholders::_1));
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localPacketCallback, this, std::placeholders::_1));
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localCameraTriggerCallback, this, std::placeholders::_1));
}

inline void DriverAdapter::start()
{
  driver_ptr_->start();
}

inline void DriverAdapter::stop()
{
  driver_ptr_->stop();
}

inline void DriverAdapter::regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback)
{
  point_cloud_cb_vec_.emplace_back(callback);
}

inline void DriverAdapter::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  scan_cb_vec_.emplace_back(callback);
}

inline void DriverAdapter::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  packet_cb_vec_.emplace_back(callback);
}

inline void DriverAdapter::regRecvCallback(const std::function<void(const ImuMsg&)>& callback)
{
  imu_cb_vec_.emplace_back(callback);
}


inline void DriverAdapter::regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
{
  camera_trigger_cb_vec_.emplace_back(callback);
}

inline void DriverAdapter::decodeScan(const ScanMsg& msg)
{
  lidar::PointCloudMsg<PointT> point_cloud_msg;
  if (driver_ptr_->decodeMsopScan(msg, point_cloud_msg))
  {
    localPointsCallback(point_cloud_msg);
  }
}

inline void DriverAdapter::decodePacket(const PacketMsg& msg)
{
  driver_ptr_->decodeDifopPkt(msg);
}

inline void DriverAdapter::localPointsCallback(const PointCloudMsg<PointT>& msg)
{
  for (auto iter : point_cloud_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(core2SDK(msg)); });
  }
}

inline void DriverAdapter::localImuCallback(const ImuMsg& msg)
{
  for (auto iter : imu_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(msg); });
  }
}


inline void DriverAdapter::localScanCallback(const ScanMsg& msg)
{
  for (auto iter : scan_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(msg); });
  }
}

inline void DriverAdapter::localPacketCallback(const PacketMsg& msg)
{
  for (auto iter : packet_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(msg); });
  }
}

inline void DriverAdapter::localCameraTriggerCallback(const CameraTrigger& msg)
{
  for (auto iter : camera_trigger_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(msg); });
  }
}

inline void DriverAdapter::localExceptionCallback(const lidar::Error& msg)
{
  switch (msg.error_code_type)
  {
    case lidar::ErrCodeType::INFO_CODE:
      TM_INFO << msg.toString() << TM_REND;
      break;
    case lidar::ErrCodeType::WARNING_CODE:
      TM_WARNING << msg.toString() << TM_REND;
      break;
    case lidar::ErrCodeType::ERROR_CODE:
      TM_ERROR << msg.toString() << TM_REND;
      break;
  }
}

inline LidarPointCloudMsg DriverAdapter::core2SDK(const lidar::PointCloudMsg<PointT>& msg)
{
  LidarPointCloudMsg::PointCloudPtr point_cloud(new LidarPointCloudMsg::PointCloud);
  point_cloud->points.assign(msg.point_cloud_ptr->begin(), msg.point_cloud_ptr->end());
  point_cloud->height = msg.height;
  point_cloud->width = msg.width;
  point_cloud->is_dense = msg.is_dense;
  LidarPointCloudMsg point_cloud_msg(point_cloud);
  point_cloud_msg.frame_id = msg.frame_id;
  point_cloud_msg.timestamp = msg.timestamp;
  point_cloud_msg.seq = msg.seq;
  return std::move(point_cloud_msg);
}

}  // namespace lidar
}  // namespace timoo
