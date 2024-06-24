cmake_minimum_required(VERSION 3.0.2)
project(web_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  geometry_msgs
)

add_message_files(
  FILES
  Float64Stamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
