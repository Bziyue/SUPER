cmake_minimum_required(VERSION 3.5)
project(rog_map)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=return-type)
cmake_policy(SET CMP0057 NEW)
cmake_policy(SET CMP0054 NEW)
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "Suppress developer warnings")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DUSE_ROS1)

string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")
message("Using ROS1")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_filters
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rog_map
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs message_filters nav_msgs pcl_ros roscpp sensor_msgs std_msgs tf2_ros visualization_msgs
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

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)
add_library(rog_map SHARED ${srcs})

target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC
  ${THIRD_PARTY}
)
