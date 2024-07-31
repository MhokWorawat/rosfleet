cmake_minimum_required(VERSION 3.0.2)
project(fleetmanagement)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  move_base_msgs
  amcl
  astar
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/gui_fleetmanagement.py
  scripts/fleetmanagement.py
  scripts/setposition.py
  scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
