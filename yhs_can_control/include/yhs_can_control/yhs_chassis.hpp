#ifndef __YHS_CHASSIS_H__
#define __YHS_CHASSIS_H__

#include "rclcpp/rclcpp.hpp"



namespace yhs_chassis
{
  inline rmw_qos_profile_t qos_profile_imu{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    2000,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  inline auto qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth),
    qos_profile_imu);

  class CanControl
  {
    public:

      explicit CanControl() {}

      virtual ~CanControl() {}

      virtual bool Run() = 0;

      virtual void Stop() = 0;

  };
}

#endif