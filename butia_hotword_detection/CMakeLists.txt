cmake_minimum_required(VERSION 2.8.3)
project(butia_hotword_detection)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED rospy std_msgs)

catkin_package(
  LIBRARIES butia_hotword_detection
  CATKIN_DEPENDS rospy std_msgs
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/detector_hotword_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

