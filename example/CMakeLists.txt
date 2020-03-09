cmake_minimum_required(VERSION 2.8.12)
project(example)

find_package(catkin REQUIRED
  network_coordinator
  protobuf_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  	network_coordinator
    protobuf_msgs
)
