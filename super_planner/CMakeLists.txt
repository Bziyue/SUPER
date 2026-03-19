cmake_minimum_required(VERSION 3.5)
project(super_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-Werror=return-type)
add_compile_options(-Werror=unused-variable)
add_compile_options(-Werror=unused-but-set-variable)
set(CMAKE_CXX_STANDARD 17)

add_definitions(-DORIGIN_AT_CORNER)
add_definitions(-DFMT_HEADER_ONLY)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
add_definitions(-DUSE_ROS1)
add_definitions(-DBACKWARD_HAS_DW=0 -DBACKWARD_HAS_BFD=0 -DBACKWARD_HAS_DWARF=0 -DBACKWARD_HAS_BACKTRACE_SYMBOL=1)

string(TOUPPER $ENV{ROS_DISTRO} ROS_VERSION)
message(STATUS "ROS version: ${ROS_VERSION}")
message("Using ROS1")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  pcl_ros
  quadrotor_msgs
  rog_map
  roscpp
  std_msgs
  tf
  visualization_msgs
)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem regex iostreams)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Curses QUIET)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES super
  CATKIN_DEPENDS geometry_msgs nav_msgs quadrotor_msgs rog_map roscpp std_msgs tf visualization_msgs
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
  ${Boost_LIBRARIES}
  yaml-cpp
)

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp)
add_library(super SHARED ${srcs})
target_include_directories(super PUBLIC
  include
)
target_link_libraries(super PUBLIC
  rog_map
  ${THIRD_PARTY}
)

add_executable(fsm_node
  Apps/fsm_node_ros1.cpp
)
target_link_libraries(fsm_node
  super
)

if(CURSES_FOUND)
  include_directories(${CURSES_INCLUDE_DIR})

  add_executable(read_replan_log
    Apps/read_replan_log.cpp
  )
  target_link_libraries(read_replan_log
    super
    ${CURSES_LIBRARIES}
    ${THIRD_PARTY}
  )

  add_executable(traj_opt_tuning
    Apps/traj_opt_tuning.cpp
  )
  target_link_libraries(traj_opt_tuning
    super
    ${CURSES_LIBRARIES}
    ${THIRD_PARTY}
  )
else()
  message(WARNING "Curses not found, skipping read_replan_log and traj_opt_tuning.")
endif()
