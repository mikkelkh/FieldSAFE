cmake_minimum_required(VERSION 2.8.3)
project(ground_truth)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/dynamic_visualizer.py
  src/static_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)