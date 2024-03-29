cmake_minimum_required(VERSION 3.0.2)
project(tme_exploration)

find_package(catkin REQUIRED COMPONENTS
  tf
  message_runtime
  roscpp
  sensor_msgs
  std_msgs
  move_base_msgs
  visualization_msgs
  geometry_msgs
  map_msgs
  nav_msgs
  actionlib_msgs
  costmap_2d
  actionlib
  multirobot_map_merge
  robot_state_publisher
  message_generation
)

add_message_files(
  FILES
  frontier.msg
  frontierArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    tf
    message_runtime
    roscpp
    sensor_msgs
    std_msgs
    move_base_msgs
    visualization_msgs
    geometry_msgs
    map_msgs
    nav_msgs
    actionlib_msgs
    costmap_2d
    actionlib
    multirobot_map_merge
    robot_state_publisher
    message_runtime
)

###########
## Build ##
###########


include_directories(${catkin_INCLUDE_DIRS} include )

add_executable(frontier_search src/costmap_client.cpp src/explore.cpp src/frontier_search.cpp)
add_dependencies(frontier_search ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontier_search ${catkin_LIBRARIES})

add_executable(map_extender src/map_extender.cpp)
add_dependencies(map_extender ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_extender ${catkin_LIBRARIES})

install(TARGETS frontier_search
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(
  PROGRAMS 
  src/analytics
  src/frontier_assign
  src/map_compare
  src/pseudo_decentralized_assign
  src/trace_removal
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)