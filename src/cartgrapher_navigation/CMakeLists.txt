cmake_minimum_required(VERSION 3.0.2)
project(cartgrapher_navigation)

find_package(catkin REQUIRED
  rospy
  roscpp
  roslib
  move_base
  actionlib
  cartographer_ros
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

find_package(cartographer REQUIRED)
include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES cartgrapher_navigation
  CATKIN_DEPENDS actionlib geometry_msgs move_base rospy roscpp tf std_msgs
  DEPENDS system_lib
)

install(DIRECTORY launch configuration_files param urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)