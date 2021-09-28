cmake_minimum_required(VERSION 3.0.2)
project(tme_exploration)


find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  move_base_msgs
)

catkin_package(
  CATKIN_DEPENDS
    costmap_2d
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    move_base_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(frontier_search src/costmap_client.cpp src/frontier_search.cpp)
add_dependencies(frontier_search ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontier_search ${catkin_LIBRARIES})

install(TARGETS frontier_search
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)