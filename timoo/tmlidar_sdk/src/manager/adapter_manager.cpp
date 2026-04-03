#include "manager/adapter_manager.h"
namespace timoo
{
namespace lidar
{
AdapterManager::~AdapterManager()
{
  stop();
}

void AdapterManager::init(const YAML::Node& config)
{
  packet_thread_flag_ = false;
  point_cloud_thread_flag_ = false;
  int msg_source = 0;
  bool send_point_cloud_ros;
  bool send_packet_ros;
  bool send_point_cloud_proto;
  bool send_packet_proto;
  bool send_camera_trigger_ros;
  bool send_imu_ros;
  std::string pcap_dir;
  double pcap_rate;
  bool pcap_repeat;
  YAML::Node common_config = yamlSubNodeAbort(config, "common");
  yamlRead<int>(common_config, "msg_source", msg_source, 0);
  yamlRead<bool>(common_config, "send_packet_ros", send_packet_ros, false);
  yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros, false);
  yamlRead<bool>(common_config, "send_point_cloud_proto", send_point_cloud_proto, false);
  yamlRead<bool>(common_config, "send_packet_proto", send_packet_proto, false);
  yamlRead<bool>(common_config, "send_camera_trigger_ros", send_camera_trigger_ros, false);
  yamlRead<bool>(common_config, "send_imu_ros", send_imu_ros, false);
  yamlRead<std::string>(common_config, "pcap_path", pcap_dir, "");
  yamlRead<double>(common_config, "pcap_rate", pcap_rate, 1);
  yamlRead<bool>(common_config, "pcap_repeat", pcap_repeat, true);
  YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
  for (uint8_t i = 0; i < lidar_config.size(); ++i)
  {
    AdapterBase::Ptr recv_ptr;
    /*Receiver*/
    switch (msg_source)
    {
      case MsgSource::MSG_FROM_LIDAR:  // use driver
        TM_INFO << "------------------------------------------------------" << TM_REND;
        TM_INFO << "Receive Packets From : Online LiDAR" << TM_REND;
        TM_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << TM_REND;
        TM_INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << TM_REND;
        TM_INFO << "------------------------------------------------------" << TM_REND;
        point_cloud_thread_flag_ = true;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_LIDAR;
        lidar_config[i]["send_imu_ros"] =send_imu_ros;
        recv_ptr = createReceiver(lidar_config[i], AdapterType::DriverAdapter);
        point_cloud_receive_adapter_vec_.push_back(recv_ptr);
        imu_receive_adapter_vec_.push_back(recv_ptr);
        if (send_packet_ros || send_packet_proto)
        {
          packet_receive_adapter_vec_.push_back(recv_ptr);
          packet_thread_flag_ = true;
        }
        break;

      case MsgSource::MSG_FROM_ROS_PACKET:  // pkt from ros
        TM_INFO << "------------------------------------------------------" << TM_REND;
        TM_INFO << "Receive Packets From : ROS" << TM_REND;
        TM_INFO << "Msop Topic: " << lidar_config[i]["ros"]["ros_recv_packet_topic"].as<std::string>() << TM_REND;
        TM_INFO << "Difop Topic: " << lidar_config[i]["ros"]["ros_recv_packet_topic"].as<std::string>() << "_difop"
                << TM_REND;
        TM_INFO << "------------------------------------------------------" << TM_REND;
        point_cloud_thread_flag_ = false;
        packet_thread_flag_ = true;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_ROS_PACKET;
        lidar_config[i]["send_imu_ros"] =send_imu_ros;
        send_packet_ros = false;
        point_cloud_receive_adapter_vec_.emplace_back(createReceiver(lidar_config[i], AdapterType::DriverAdapter));
        packet_receive_adapter_vec_.emplace_back(createReceiver(lidar_config[i], AdapterType::PacketRosAdapter));
        packet_receive_adapter_vec_[i]->regRecvCallback(
            std::bind(&AdapterBase::decodeScan, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
        packet_receive_adapter_vec_[i]->regRecvCallback(
            std::bind(&AdapterBase::decodePacket, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
        break;

      case MsgSource::MSG_FROM_PCAP:  // pcap
        TM_INFO << "------------------------------------------------------" << TM_REND;
        TM_INFO << "Receive Packets From : Pcap" << TM_REND;
        TM_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << TM_REND;
        TM_INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << TM_REND;
        TM_INFO << "------------------------------------------------------" << TM_REND;
        point_cloud_thread_flag_ = true;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PCAP;
        lidar_config[i]["send_imu_ros"] =send_imu_ros;
        lidar_config[i]["driver"]["read_pcap"] = true;
        lidar_config[i]["driver"]["pcap_path"] = pcap_dir;
        lidar_config[i]["driver"]["pcap_rate"] = pcap_rate;
        lidar_config[i]["driver"]["pcap_repeat"] = pcap_repeat;
        recv_ptr = createReceiver(lidar_config[i], AdapterType::DriverAdapter);
        point_cloud_receive_adapter_vec_.push_back(recv_ptr);
        imu_receive_adapter_vec_.push_back(recv_ptr);
        if (send_packet_ros || send_packet_proto)
        {
          packet_receive_adapter_vec_.push_back(recv_ptr);
          packet_thread_flag_ = true;
        }
        break;

      case MsgSource::MSG_FROM_PROTO_PACKET:  // packets from proto
        TM_INFO << "------------------------------------------------------" << TM_REND;
        TM_INFO << "Receive Packets From : Protobuf-UDP" << TM_REND;
        TM_INFO << "Msop Port: " << lidar_config[i]["proto"]["msop_recv_port"].as<uint16_t>() << TM_REND;
        TM_INFO << "Difop Port: " << lidar_config[i]["proto"]["difop_recv_port"].as<uint16_t>() << TM_REND;
        TM_INFO << "------------------------------------------------------" << TM_REND;
        point_cloud_thread_flag_ = false;
        packet_thread_flag_ = true;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PROTO_PACKET;
        lidar_config[i]["send_imu_ros"] =send_imu_ros;
        send_packet_proto = false;
        point_cloud_receive_adapter_vec_.emplace_back(createReceiver(lidar_config[i], AdapterType::DriverAdapter));
        packet_receive_adapter_vec_.emplace_back(createReceiver(lidar_config[i], AdapterType::PacketProtoAdapter));
        packet_receive_adapter_vec_[i]->regRecvCallback(
            std::bind(&AdapterBase::decodeScan, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
        packet_receive_adapter_vec_[i]->regRecvCallback(
            std::bind(&AdapterBase::decodePacket, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
        break;

      case MsgSource::MSG_FROM_PROTO_POINTCLOUD:  // point cloud from proto
        TM_INFO << "------------------------------------------------------" << TM_REND;
        TM_INFO << "Receive PointCloud From : Protobuf-UDP" << TM_REND;
        TM_INFO << "PointCloud Port: " << lidar_config[i]["proto"]["point_cloud_recv_port"].as<uint16_t>() << TM_REND;
        TM_INFO << "------------------------------------------------------" << TM_REND;
        point_cloud_thread_flag_ = true;
        packet_thread_flag_ = false;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PROTO_POINTCLOUD;
        send_point_cloud_proto = false;
        send_packet_ros = false;
        send_packet_proto = false;
        send_camera_trigger_ros = false;
        point_cloud_receive_adapter_vec_.emplace_back(
            createReceiver(lidar_config[i], AdapterType::PointCloudProtoAdapter));
        packet_receive_adapter_vec_.emplace_back(nullptr);
        break;

      default:
        TM_ERROR << "Not use LiDAR or Wrong LiDAR message source! Abort!" << TM_REND;
        exit(-1);
    }

    /*Transmitter*/
    if (send_packet_ros)
    {
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      TM_DEBUG << "Send Packets To : ROS" << TM_REND;
      TM_DEBUG << "Msop Topic: " << lidar_config[i]["ros"]["ros_send_packet_topic"].as<std::string>() << TM_REND;
      TM_DEBUG << "Difop Topic: " << lidar_config[i]["ros"]["ros_send_packet_topic"].as<std::string>() << "_difop"
               << TM_REND;
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      lidar_config[i]["send_packet_ros"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::PacketRosAdapter);
      packet_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      packet_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendScan, transmitter_ptr, std::placeholders::_1));
      packet_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendPacket, transmitter_ptr, std::placeholders::_1));
    }
    if (send_packet_proto)
    {
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      TM_DEBUG << "Send Packets To : Protobuf-UDP" << TM_REND;
      TM_DEBUG << "Msop Port:  " << lidar_config[i]["proto"]["msop_send_port"].as<uint16_t>() << TM_REND;
      TM_DEBUG << "Difop Port: " << lidar_config[i]["proto"]["difop_send_port"].as<uint16_t>() << TM_REND;
      TM_DEBUG << "Target IP: " << lidar_config[i]["proto"]["packet_send_ip"].as<std::string>() << TM_REND;
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      lidar_config[i]["send_packet_proto"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::PacketProtoAdapter);
      packet_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      packet_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendScan, transmitter_ptr, std::placeholders::_1));
      packet_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendPacket, transmitter_ptr, std::placeholders::_1));
    }
    if (send_point_cloud_ros)
    {
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      TM_DEBUG << "Send PointCloud To : ROS" << TM_REND;
      TM_DEBUG << "PointCloud Topic: " << lidar_config[i]["ros"]["ros_send_point_cloud_topic"].as<std::string>()
               << TM_REND;
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      lidar_config[i]["send_point_cloud_ros"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::PointCloudRosAdapter);
      point_cloud_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      point_cloud_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendPointCloud, transmitter_ptr, std::placeholders::_1));
    }
    if (send_point_cloud_proto)
    {
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      TM_DEBUG << "Send PointCloud To : Protobuf-UDP" << TM_REND;
      TM_DEBUG << "PointCloud Port:  " << lidar_config[i]["proto"]["point_cloud_send_port"].as<uint16_t>() << TM_REND;
      TM_DEBUG << "Target IP: " << lidar_config[i]["proto"]["point_cloud_send_ip"].as<std::string>() << TM_REND;
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      lidar_config[i]["send_point_cloud_proto"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::PointCloudProtoAdapter);
      point_cloud_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      point_cloud_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendPointCloud, transmitter_ptr, std::placeholders::_1));
    }
    if (send_camera_trigger_ros)
    {
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      TM_DEBUG << "Send Camera Trigger To : ROS" << TM_REND;
      for (auto iter : lidar_config[i]["camera"])
      {
        TM_DEBUG << "Camera : " << iter["frame_id"].as<std::string>()
                 << "  Trigger Angle : " << iter["trigger_angle"].as<double>() << TM_REND;
      }
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::CameraTriggerRosAdapter);
      point_cloud_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendCameraTrigger, transmitter_ptr, std::placeholders::_1));
    }
    if(send_imu_ros){
      TM_DEBUG << "------------------------------------------------------" << TM_REND;
      TM_DEBUG << "Send Packet To : ROS" << TM_REND;
      TM_DEBUG << "IMU Topic: " << lidar_config[i]["ros"]["ros_send_imu_topic"].as<std::string>()
               << TM_REND;
      TM_DEBUG << "------------------------------------------------------" << TM_REND;  
      lidar_config[i]["send_imu_ros"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i],AdapterType::ImuAdapter);     
      imu_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      imu_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendImu, transmitter_ptr, std::placeholders::_1)); 
    }
  }
}

void AdapterManager::start()
{
  if (point_cloud_thread_flag_)
  {
    for (auto& iter : point_cloud_receive_adapter_vec_)
    {
      if (iter != nullptr)
        iter->start();
    }
  }
  if (packet_thread_flag_)
  {
    for (auto& iter : packet_receive_adapter_vec_)
    {
      if (iter != nullptr)
      {
        iter->start();
      }
    }
  }
}

void AdapterManager::stop()
{
  if (point_cloud_thread_flag_)
  {
    for (auto& iter : point_cloud_receive_adapter_vec_)
    {
      if (iter != nullptr)
        iter->stop();
    }
  }
  if (packet_thread_flag_)
  {
    for (auto& iter : packet_receive_adapter_vec_)
    {
      if (iter != nullptr)
      {
        iter->stop();
      }
    }
  }
}

std::shared_ptr<AdapterBase> AdapterManager::createReceiver(const YAML::Node& config, const AdapterType& adapter_type)
{
  std::shared_ptr<AdapterBase> receiver;
  switch (adapter_type)
  {
    case AdapterType::DriverAdapter:
      receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<DriverAdapter>());
      receiver->init(config);
      break;

    case AdapterType::PacketRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
      receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PacketRosAdapter>());
      receiver->init(config);
      break;
#else
      TM_ERROR << "ROS not found! Could not use ROS functions!" << TM_REND;
      exit(-1);
#endif

    case AdapterType::PacketProtoAdapter:
#ifdef PROTO_FOUND
      receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PacketProtoAdapter>());
      receiver->init(config);
      break;
#else
      TM_ERROR << "Protobuf not found! Could not use Protobuf functions!" << TM_REND;
      exit(-1);
#endif

    case AdapterType::PointCloudProtoAdapter:
#ifdef PROTO_FOUND
      receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PointCloudProtoAdapter>());
      receiver->init(config);
      break;
#else
      TM_ERROR << "Protobuf not found! Could not use Protobuf functions!" << TM_REND;
      exit(-1);
#endif

    default:
      TM_ERROR << "Create receiver failed. Abort!" << TM_REND;
      exit(-1);
  }

  return receiver;
}

std::shared_ptr<AdapterBase> AdapterManager::createTransmitter(const YAML::Node& config,
                                                               const AdapterType& adapter_type)
{
  std::shared_ptr<AdapterBase> transmitter;
  switch (adapter_type)
  {
    case AdapterType::PacketRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PacketRosAdapter>());
      transmitter->init(config);
      break; 
#else
      TM_ERROR << "ROS not found! Could not use ROS functions!" << TM_REND;
      exit(-1);
#endif

    case AdapterType::PacketProtoAdapter:
#ifdef PROTO_FOUND
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PacketProtoAdapter>());
      transmitter->init(config);
      break;
#else
      TM_ERROR << "Protobuf not found! Could not use Protobuf functions!" << TM_REND;
      exit(-1);
#endif

    case AdapterType::PointCloudRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PointCloudRosAdapter>());
      transmitter->init(config);
      break;
#else
      TM_ERROR << "ROS not found! Could not use ROS functions!" << TM_REND;
      exit(-1);
#endif

    case AdapterType::PointCloudProtoAdapter:
#ifdef PROTO_FOUND
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PointCloudProtoAdapter>());
      transmitter->init(config);
      break;
#else
      TM_ERROR << "Protobuf not found! Could not use Protobuf functions!" << TM_REND;
      exit(-1);
#endif

    case AdapterType::ImuAdapter:
#if (ROS_FOUND || ROS2_FOUND) 
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<ImuAdapter>());
      transmitter->init(config);
      break;
#else
      TM_ERROR << "ROS not found! Could not use ROS functions!" << TM_REND;
      exit(-1);
#endif

    case AdapterType::CameraTriggerRosAdapter:
#if (ROS_FOUND)  //< Camera trigger not support ROS2 yet
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<CameraTriggerRosAdapter>());
      transmitter->init(config);
      break;
#else
      TM_ERROR << "ROS not found! Could not use ROS functions!" << TM_REND;
      exit(-1);
#endif



    default:
      TM_ERROR << "Create transmitter failed. Abort!" << TM_REND;
      exit(-1);
  }

  return transmitter;
}

}  // namespace lidar
}  // namespace timoo
