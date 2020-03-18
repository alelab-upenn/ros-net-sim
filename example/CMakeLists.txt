cmake_minimum_required(VERSION 2.8.12)
project(example)

set(DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
  gazebo_msgs
  nav_msgs
  network_coordinator
  nodelet
  protobuf_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(catkin REQUIRED
  ${DEPENDS}
  message_generation
)

add_action_files(
  FILES
  WaypointNavigation.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  # not designed to be used by other packages
  # INCLUDE_DIRS include
  # LIBRARIES waypoint_nav_nodelet
  CATKIN_DEPENDS
    ${DEPENDS}
    message_runtime
    turtlebot3_gazebo
    turtlebot3_bringup
    octomap_server
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(nav_base_nodelet src/nav_base.cpp src/nav_base_nodelet.cpp)
add_dependencies(nav_base_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(nav_base_nodelet ${catkin_LIBRARIES})

add_executable(nav_base_node src/nav_base_node.cpp)
add_dependencies(nav_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(nav_base_node ${catkin_LIBRARIES} nav_base_nodelet)
