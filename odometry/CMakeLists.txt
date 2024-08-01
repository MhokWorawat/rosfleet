cmake_minimum_required(VERSION 3.0.2)
project(odometry)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

# Uncomment this section if your package has messages, services, or actions
# and adjust accordingly.
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

# Uncomment this section if your package has dynamic reconfigure options
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odometry_setup
  CATKIN_DEPENDS roscpp tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# Uncomment and adjust if your package has libraries
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/odometry_setup.cpp
# )

## Add cmake target dependencies of the library
# Uncomment and adjust if your package has libraries
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare C++ executables
add_executable(agv01_odom src/agv01_odom.cpp)
add_executable(agv02_odom src/agv02_odom.cpp)
add_executable(agv03_odom src/agv03_odom.cpp)
add_executable(agv04_odom src/agv04_odom.cpp)

## Add cmake target dependencies of the executable
# Uncomment and adjust if your package has dependencies
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(agv01_odom
  ${catkin_LIBRARIES}
)
target_link_libraries(agv02_odom
  ${catkin_LIBRARIES}
)
target_link_libraries(agv03_odom
  ${catkin_LIBRARIES}
)
target_link_libraries(agv04_odom
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS agv01_odom agv02_odom agv03_odom agv04_odom
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# Uncomment and adjust if you have other files to install
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# Uncomment and adjust if you have tests
# catkin_add_gtest(${PROJECT_NAME}-test test/test_odometry_setup.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# Uncomment and adjust if you have python tests
# catkin_add_nosetests(test)
