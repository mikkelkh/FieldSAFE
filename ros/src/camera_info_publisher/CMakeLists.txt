cmake_minimum_required(VERSION 2.8.3)
project(camera_info_publisher)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/camera_info_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

