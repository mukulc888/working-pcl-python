cmake_minimum_required(VERSION 3.18.0)
project(robot_control)


find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  diagnostic_msgs
  diagnostic_updater
)

catkin_package(
 CATKIN_DEPENDS ackermann_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_control_node nodes/robot_control.cpp)

target_link_libraries(robot_control_node
   ${catkin_LIBRARIES}
)
