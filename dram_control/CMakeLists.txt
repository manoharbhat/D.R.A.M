cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 2.8.3)
project(dram_control)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  hardware_interface
  controller_manager
  sensor_msgs
  serial
  nav_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS hardware_interface controller_manager sensor_msgs
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/dram_hardware_interface.cpp src/dram_hardware_interface_node.cpp )
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME node PREFIX "")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

