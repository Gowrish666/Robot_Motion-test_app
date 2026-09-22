cmake_minimum_required(VERSION 3.0.2)
project(robot_interface)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  robot_motion_test_msgs
  roscpp
  robot_motion_test_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    geometry_msgs
    nav_msgs
    robot_motion_test_msgs
    roscpp
  robot_motion_test_msgs
    std_msgs
)

include_directories(
  /root/anscer_ws/devel/include
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_interface_node
  src/robot_interface_node.cpp
  src/odom_processor.cpp
  src/robot_adapter.cpp
)

add_dependencies(
  robot_interface_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  robot_interface_node
  ${catkin_LIBRARIES}
)

install(
  TARGETS robot_interface_node
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)