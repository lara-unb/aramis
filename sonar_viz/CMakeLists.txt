cmake_minimum_required(VERSION 2.8.3)
project(sonar_viz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  p2os_msgs
  tf
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES sonar_viz
 CATKIN_DEPENDS roscpp sensor_msgs tf
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)

add_executable(sonar_viz src/sonar_viz.cpp)
target_link_libraries(sonar_viz ${catkin_LIBRARIES})

# add_executable(la src/la.cpp)
# target_link_libraries(la ${catkin_LIBRARIES})
