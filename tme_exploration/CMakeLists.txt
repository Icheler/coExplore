cmake_minimum_required(VERSION 3.0.2)
project(tme_exploration)


find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  move_base_msgs
)
set(YOUR_DIRECTORY /opt/ros/noetic/share/costmap_2d)
include_directories( ${YOUR_DIRECTORY})
set(SOURCES ${YOUR_DIRECTORY}/costmap_2d/costmap_2d.h )

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(frontier_search src/frontier_search.cpp src/costmap_client.cpp)
target_link_libraries(frontier_search ${catkin_LIBRARIES})