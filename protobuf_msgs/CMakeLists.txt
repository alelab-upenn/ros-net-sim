cmake_minimum_required(VERSION 2.8.12)
project(protobuf_msgs)

find_package(catkin REQUIRED)
find_package(Protobuf REQUIRED)

catkin_python_setup()

# INCLUDE_DIRS must exist before calling catkin_package; later, generated
# protobuf headers will be placed in this directory. Setting INCLUDE_DIRS in
# this way ensures generated probobuf headers can be included in other packages
# via find_package() in the usual catkin way.
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
catkin_package(
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  LIBRARIES ${PROJECT_NAME}_proto
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
)

set(PROTO_FILES
  protobuf/channel_data.proto
  protobuf/physics_update.proto
  protobuf/network_update.proto
)
# creates custom commands in cmake to generate protobuf code, if included in a
# target
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
protobuf_generate_python(PROTO_PY ${PROTO_FILES})

# triggers the generation of protobuf C++ code
add_library(${PROJECT_NAME}_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(${PROJECT_NAME}_proto ${Protobuf_LIBRARIES})

# triggers the generation of protobuf python code
add_custom_target(${PROJECT_NAME}_proto_python ALL DEPENDS ${PROTO_PY})

# Copy protobuf generated headers from the build space to the devel space so
# that they can be used by other catkin packages. Note that the copying of
# generated protobuf headers must happen after they are built, which is only
# triggered when they are included in a target. This behavior is achived by
# setting the TARGET field of add_custom_command() appropriately.
set(PROTO_HDRS_DEVEL_DIR ${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/)
file(MAKE_DIRECTORY ${PROTO_HDRS_DEVEL_DIR})
foreach(FIL ${PROTO_HDRS})
  add_custom_command(
    TARGET ${PROJECT_NAME}_proto
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${FIL} ${PROTO_HDRS_DEVEL_DIR}
    COMMENT "copying ${FIL} to ${PROTO_HDRS_DEVEL_DIR}"
    DEPENDS ${FIL}
    VERBATIM)
endforeach()

# Copy protobuf generated python code from the build space to the devel space so
# that it can be used by other catkin packages. Note that the copying of
# generated protobuf python code must happen after it is "built", which is only
# triggered when it is included in a target. This behavior is achived by setting
# the TARGET field of add_custom_command() appropriately.
set(PROTO_PYTHON_DEVEL_DIR ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
file(MAKE_DIRECTORY ${PROTO_PYTHON_DEVEL_DIR})
set(PYTHON_INIT_FILEPATH "${PROTO_PYTHON_DEVEL_DIR}/__init__.py")
if(NOT EXISTS ${PYTHON_INIT_FILEPATH})
  add_custom_target(create_empty_init ALL
    COMMAND ${CMAKE_COMMAND} -E touch ${PYTHON_INIT_FILEPATH}
    COMMENT "Created empty __init__.py."
    VERBATIM)
endif()
foreach(FIL ${PROTO_PY})
  add_custom_command(
    TARGET ${PROJECT_NAME}_proto_python
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${FIL} ${PROTO_PYTHON_DEVEL_DIR}
    COMMENT "copying ${FIL} to ${PROTO_PYTHON_DEVEL_DIR}"
    DEPENDS ${FIL}
    VERBATIM)
endforeach()
