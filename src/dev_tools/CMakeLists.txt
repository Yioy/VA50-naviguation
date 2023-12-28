cmake_minimum_required(VERSION 3.0.2)
project(dev_tools)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp tf2_ros sensor_msgs tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_to_twist_node src/tf_to_twist.cpp)

target_link_libraries(tf_to_twist_node
  ${catkin_LIBRARIES}
)