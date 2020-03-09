cmake_minimum_required(VERSION 2.8.12)
project(network_coordinator)

find_package(catkin REQUIRED
  protobuf_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    protobuf_msgs
)
