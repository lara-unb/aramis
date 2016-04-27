cmake_minimum_required(VERSION 2.8.3)
project(aramis_vrep)

find_package(catkin REQUIRED COMPONENTS
  aramis_description
  move_base
  vrep_odom_pioneer
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aramis_vrep
#  CATKIN_DEPENDS aramis_description move_base vrep_odom_pioneer
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

