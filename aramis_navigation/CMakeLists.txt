cmake_minimum_required(VERSION 2.8.3)
project(aramis_navigation)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  roscpp
  rospy
  tf
  message_filters
  move_base_msgs
  actionlib
  angles
  laser_assembler
  actionlib
  control_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES radom_walk
 # CATKIN_DEPENDS roslaunch sensor_msgs
 # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(random_walk src/random_walk.cpp)
target_link_libraries(random_walk ${catkin_LIBRARIES})

add_executable(random_walk_node src/random_walk_node.cpp)
target_link_libraries(random_walk_node random_walk ${catkin_LIBRARIES})
