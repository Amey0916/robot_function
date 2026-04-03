# - Config file for the  package
# It defines the following variables
#  tm_driver_INCLUDE_DIRS - include directories for 
#  tm_driver_LIBRARIES    - libraries to link against
#  tm_driver_FOUND        - found flag

if(WIN32)
  if(CMAKE_SIZEOF_VOID_P EQUAL 8) # 64-bit
    set(Boost_ARCHITECTURE "-x64")
  elseif(CMAKE_SIZEOF_VOID_P EQUAL 4) # 32-bit
    set(Boost_ARCHITECTURE "-x32")
  endif()
  set(Boost_USE_STATIC_LIBS ON)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
endif(WIN32)

if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")
endif(${ENABLE_TRANSFORM})

set(tm_driver_INCLUDE_DIRS "/home/yhs/ros2_ws/src/hardware/timoo/tmlidar_sdk/src/tm_driver/src;/usr/local/tmlidar_sdk/include")
set(TM_DRIVER_INCLUDE_DIRS "/home/yhs/ros2_ws/src/hardware/timoo/tmlidar_sdk/src/tm_driver/src;/usr/local/tmlidar_sdk/include")

set(tm_driver_LIBRARIES "Boost::system;Boost::date_time;Boost::regex;pcap")
set(TM_DRIVER_LIBRARIES "Boost::system;Boost::date_time;Boost::regex;pcap")

set(tm_driver_FOUND true)
set(TM_DRIVER_FOUND true)
