cmake_minimum_required(VERSION 2.8.3)
project(snapdragon_simple_autonomy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DMAVLINK_DIALECT=common")
add_definitions(-D_ROS)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslaunch
  rospy
  std_msgs
)

# Check launch files
roslaunch_add_file_check(launch)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    roslaunch
    rospy
    std_msgs
)

include_directories(
  include
  include/simple_autonomy
  ${catkin_INCLUDE_DIRS}
)

add_executable(snapdragon_simple_autonomy_node
  simple_autonomy.cpp
)

message("${CMAKE_CURRENT_SOURCE_DIR}")

find_library(SIMPLE_AUTONOMY simple_autonomy_apps
  NAMES libsimple_autonomy_apps
  HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

if(NOT SIMPLE_AUTONOMY)
  message(FATAL_ERROR "simple_autonomy_apps library not found")
endif()

target_link_libraries(snapdragon_simple_autonomy_node
  ${catkin_LIBRARIES}
  ${SIMPLE_AUTONOMY}
)
