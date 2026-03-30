cmake_minimum_required(VERSION 3.8)
project(yhs_can_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-pedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yhs_can_interfaces REQUIRED)

include_directories(
  include)


add_executable(yhs_can_control_node src/main.cpp src/yhs_dgt_can_control_node.cpp src/yhs_fw_can_control_node.cpp
src/yhs_fr_can_control_node.cpp src/yhs_mk_can_control_node.cpp)

ament_target_dependencies(
  yhs_can_control_node
  tf2_ros
  std_msgs
  rclcpp
  nav2_msgs
  tf2_msgs
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  yhs_can_interfaces
)

install(TARGETS yhs_can_control_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch params
    DESTINATION share/${PROJECT_NAME})
    
install(DIRECTORY include/
   DESTINATION include/
)
    
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
