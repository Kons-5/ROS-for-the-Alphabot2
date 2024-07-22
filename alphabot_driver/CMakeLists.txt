cmake_minimum_required(VERSION 3.0.2)
project(alphabot_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/alphabot_driver/AlphaBot.py
  src/alphabot_driver/PCA9685.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

