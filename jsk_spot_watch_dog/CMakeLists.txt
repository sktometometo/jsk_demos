cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_watch_dog)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/watch-dog.l
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
