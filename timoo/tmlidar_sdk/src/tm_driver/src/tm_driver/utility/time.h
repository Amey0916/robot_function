#pragma once
#include <tm_driver/common/common_header.h>
namespace timoo
{
namespace lidar
{
inline double getTime(void)
{
  const auto t = std::chrono::system_clock::now();
  const auto t_sec = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch());
  return (double)t_sec.count();
}
}  // namespace lidar
}  // namespace timoo