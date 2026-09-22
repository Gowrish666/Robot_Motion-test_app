cmake_minimum_required(VERSION 3.0.2)
project(robot_motion_test_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
)

add_service_files(
  FILES
    MoveToWaypoint.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    roscpp
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
