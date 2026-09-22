cmake_minimum_required(VERSION 3.0.2)

project(robot_motion_test_data)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robot_motion_test_data
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /opt/mongo-cxx-driver/include/mongocxx/v_noabi
  /opt/mongo-cxx-driver/include/bsoncxx/v_noabi
)

add_library(robot_motion_test_data
  src/data_access_layer.cpp
)

target_link_libraries(robot_motion_test_data
  ${catkin_LIBRARIES}
  mongocxx
  bsoncxx
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS robot_motion_test_data
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)