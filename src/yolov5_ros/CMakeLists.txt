cmake_minimum_required(VERSION 3.0.2)
project(yolov5_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  detection_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)