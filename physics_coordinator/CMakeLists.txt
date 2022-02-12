cmake_minimum_required(VERSION 2.8.3)
project(physics_coordinator)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED
  channel_simulator
  gazebo_ros
  protobuf_msgs
  roscpp
)

find_package(Boost REQUIRED system iostreams)

catkin_package(
  CATKIN_DEPENDS
    channel_simulator
    gazebo_ros
    protobuf_msgs
    roscpp
  DEPENDS
    Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

include(FindPkgConfig)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
string(REPLACE ";" " " GAZEBO_CFLAGS "${GAZEBO_CFLAGS}")
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CFLAGS}")

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(gazebo_custom_main src/gazebo_custom_main.cpp)
add_dependencies(gazebo_custom_main ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_custom_main ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

configure_file(scripts/gazebo_custom_server ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/gazebo_custom_server COPYONLY)
configure_file(scripts/gazebo_custom_server_debug ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/gazebo_custom_server_debug COPYONLY)
configure_file(scripts/libcommon.sh ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/libcommon.sh COPYONLY)
