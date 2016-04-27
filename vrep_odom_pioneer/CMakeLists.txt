cmake_minimum_required(VERSION 2.8.3)
project(vrep_odom_pioneer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf
)
catkin_package(
 # INCLUDE_DIRS include
#  LIBRARIES vrep_odom_pioneer
 CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom src/odom.cpp)
target_link_libraries(odom
  ${catkin_LIBRARIES}
)
