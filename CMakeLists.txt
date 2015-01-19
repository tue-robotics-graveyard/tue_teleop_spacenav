cmake_minimum_required(VERSION 2.8.3)
project(tue_teleop_spacenav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(spacenav_publisher src/spacenav_publisher.cpp)
target_link_libraries(spacenav_publisher ${catkin_LIBRARIES})

add_executable(spacenav_publisher_base_only src/spacenav_publisher_base_only.cpp)
target_link_libraries(spacenav_publisher_base_only ${catkin_LIBRARIES})
