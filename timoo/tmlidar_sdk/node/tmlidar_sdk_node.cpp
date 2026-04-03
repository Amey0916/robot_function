/*********************************************************************************************************************
Copyright (c) 2020 Timoo
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For Timoo LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the Timoo, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#include <signal.h>
#include "manager/adapter_manager.h"
using namespace timoo::lidar;
std::mutex g_mtx;
std::condition_variable g_cv;
static void sigHandler(int sig)
{
  TM_MSG << "Timoo-LiDAR-Driver is stopping....." << TM_REND;
#ifdef ROS_FOUND
  ros::shutdown();
#endif
  g_cv.notify_all();
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);  ///< bind ctrl+c signal with the sigHandler function
  TM_TITLE << "********************************************************" << TM_REND;
  TM_TITLE << "**********    TIMOOTECH LIDAR SDK Version: v" << TMLIDAR_VERSION_MAJOR << "." << TMLIDAR_VERSION_MINOR << "."
           << TMLIDAR_VERSION_PATCH << "     **********" << TM_REND;
  TM_TITLE << "********************************************************" << TM_REND;

  std::shared_ptr<AdapterManager> demo_ptr = std::make_shared<AdapterManager>();
  YAML::Node config;
  try
  {
    config = YAML::LoadFile((std::string)PROJECT_PATH + "/config/config.yaml");
  }
  catch (...)
  {
    TM_ERROR << "Config file format wrong! Please check the format(e.g. indentation) " << TM_REND;
    return -1;
  }

#ifdef ROS_FOUND  ///< if ROS is found, call the ros::init function
  ros::init(argc, argv, "tmlidar_sdk_node", ros::init_options::NoSigintHandler);
#endif

#ifdef ROS2_FOUND  ///< if ROS2 is found, call the rclcpp::init function
  rclcpp::init(argc, argv);
#endif

  demo_ptr->init(config);
  demo_ptr->start();
  TM_MSG << "Timoo-LiDAR-Driver is running....." << TM_REND;

#ifdef ROS_FOUND
  ros::spin();
#else
  std::unique_lock<std::mutex> lck(g_mtx);
  g_cv.wait(lck);
#endif
  return 0;
}