cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DUSE_ROS1)
add_definitions(-DBACKWARD_HAS_DW=0 -DBACKWARD_HAS_BFD=0 -DBACKWARD_HAS_DWARF=0 -DBACKWARD_HAS_BACKTRACE_SYMBOL=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_filters
  mavros_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs mavros_msgs
)

include_directories(
  SYSTEM
  include
  ../../../devel/include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  yaml-cpp
)

add_executable(waypoint_mission
  Apps/ros1_waypoint_mission.cpp
)

target_link_libraries(waypoint_mission
  ${THIRD_PARTY}
)
