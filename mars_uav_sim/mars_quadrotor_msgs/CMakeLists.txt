cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_message_files(
  FILES
  PositionCommand.msg
  MpcPositionCommand.msg
  PolynomialTrajectory.msg
  TrakingPerformance.msg
  QuadrotorState.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs message_runtime
)
