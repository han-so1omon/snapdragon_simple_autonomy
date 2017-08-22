cmake_minimum_required(VERSION 2.8.3)
project(snapdragon_mavros_autonomy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -DMAVLINK_DIALECT=common")
add_definitions(-D_ROS)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  roslaunch
  rospy
  std_msgs
  mavros_msgs
)

find_package(Boost REQUIRED COMPONENTS
  system
)

set(OKVIS_INSTALLATION /usr/local)

find_package(OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS})

find_package( okvis 1.1 REQUIRED )
include_directories(${OKVIS_INCLUDE_DIRS})

list(APPEND CMAKE_PREFIX_PATH ${OKVIS_INSTALLATION})
find_package( Ceres REQUIRED )
include_directories(${CERES_INCLUDE_DIRS})

list(APPEND CMAKE_MODULE_PATH ${OKVIS_INSTALLATION}/lib/CMake)
find_package(VISensor)
if(VISENSORDRIVER_FOUND)
  message(STATUS "Found libvisensor.")
else()
  message(STATUS "libvisensor not found")
endif()

# Check launch files
roslaunch_add_file_check(launch)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    roslaunch
    rospy
    std_msgs
    mavros_msgs
)

include_directories(
  include
  ${OKVIS_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(snapdragon_mavros_autonomy_node
  mavros_autonomy.cpp
)

target_link_libraries(snapdragon_mavros_autonomy_node
  ${OKVIS_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
)

if(${VISENSORDRIVER_FOUND})
  target_link_libraries(snapdragon_mavros_autonomy_node
    ${VISensorDriver_LIBRARY}
  )
endif()

add_executable(okvis_pose_node
  okvis_pose.cpp
)

target_link_libraries(okvis_pose_node
  pthread
  ${OKVIS_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

if(${VISENSORDRIVER_FOUND})
  target_link_libraries(okvis_pose_node ${VISensorDriver_LIBRARY})
endif()
