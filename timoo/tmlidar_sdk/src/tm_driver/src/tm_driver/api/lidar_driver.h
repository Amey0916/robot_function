#pragma once
#include <tm_driver/driver/lidar_driver_impl.hpp>

namespace timoo
{
namespace lidar
{
/**
 * @brief This is the Timoo LiDAR driver interface class
 */
template <typename PointT>
class LidarDriver
{
public:
  /**
   * @brief Constructor, instanciate the driver pointer
   */
  LidarDriver():driver_ptr_(std::make_shared<LidarDriverImpl<PointT>>())
  {
  }

  /**
   * @brief Deconstructor, stop all threads
   */
  ~LidarDriver()
  {
    stop();
  }

  /**
   * @brief The initialization function, used to set up parameters and instance objects,
   *        used when get packets from online lidar or pcap
   * @param param The custom struct TMDriverParam
   * @return If successful, return true; else return false
   */
  inline bool init(const TMDriverParam& param )
  {
    return driver_ptr_->init(param );
  }

  /**
   * @brief The initialization function which only initialize decoder(not include input module). If lidar packets are
   * from ROS or other ways excluding online lidar and pcap, call this function to initialize instead of calling init()
   * @param param The custom struct TMDriverParam
   */
  inline void initDecoderOnly(const TMDriverParam& param)
  {
    driver_ptr_->initDecoderOnly(param);
  }

  /**
   * @brief Start the thread to receive and decode packets
   * @return If successful, return true; else return false
   */
  inline bool start()
  {
    return driver_ptr_->start();
  }

  /**
   * @brief Stop all threads
   */
  inline void stop()
  {
    driver_ptr_->stop();
  }

  /**
   * @brief Register the lidar point cloud callback function to driver. When point cloud is ready, this function will be
   * called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const PointCloudMsg<PointT>&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the lidar scan message callback function to driver.When lidar scan message is ready, this function
   * will be called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the imu message callback function to driver.When lidar scan message is ready, this function
   * will be called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const ImuMsg&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the lidar difop packet message callback function to driver. When lidar difop packet message is
   * ready, this function will be called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the camera trigger message callback function to driver. When trigger message is ready, this
   * function will be called
   * @param callback The callback function
   */
  inline void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
  {
    driver_ptr_->regRecvCallback(callback);
  }

  /**
   * @brief Register the exception message callback function to driver. When error occurs, this function will be called
   * @param callback The callback function
   */
  inline void regExceptionCallback(const std::function<void(const Error&)>& callback)
  {
    driver_ptr_->regExceptionCallback(callback);
  }

  /**
   * @brief Get the current lidar temperature
   * @param input_temperature The variable to store lidar temperature
   * @return if get temperature successfully, return true; else return false
   */
  inline bool getLidarTemperature(double& input_temperature)
  {
    return driver_ptr_->getLidarTemperature(input_temperature);
  }

  /**
   * @brief Decode lidar scan messages to point cloud
   * @note This function will only work after decodeDifopPkt is called unless wait_for_difop is set to false
   * @param pkt_scan_msg The lidar scan message
   * @param point_cloud_msg The output point cloud message
   * @return if decode successfully, return true; else return false
   */
  inline bool decodeMsopScan(const ScanMsg& pkt_scan_msg, PointCloudMsg<PointT>& point_msg)
  {
    return driver_ptr_->decodeMsopScan(pkt_scan_msg, point_msg);
  }

  /**
   * @brief Decode lidar difop messages
   * @param pkt_msg The lidar difop packet
   */
  inline void decodeDifopPkt(const PacketMsg& pkt_msg)
  {
    driver_ptr_->decodeDifopPkt(pkt_msg);
  }

private:
  std::shared_ptr<LidarDriverImpl<PointT>> driver_ptr_;  ///< The driver pointer
};

}  // namespace lidar
}  // namespace timoo
