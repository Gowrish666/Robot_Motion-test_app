cmake_minimum_required(VERSION 3.0.2)

project(robot_motion_test_app)

add_compile_options(-std=c++14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_AUTOUIC_SEARCH_PATHS
    ${CMAKE_CURRENT_SOURCE_DIR}/ui
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    actionlib_msgs
    dynamic_reconfigure
    graph_msgs
    mission_msgs
    robot_motion_test_msgs
    robot_motion_test_data
)

find_package(Qt5 REQUIRED COMPONENTS
    Widgets
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        actionlib_msgs
        dynamic_reconfigure
        graph_msgs
        mission_msgs
        robot_motion_test_msgs
        robot_motion_test_data
)

include_directories(
    include

    ${catkin_INCLUDE_DIRS}

    # Anscer workspace generated message headers
    /root/anscer_ws/devel/include

    # robot_motion_test_data headers
    ${robot_motion_test_data_INCLUDE_DIRS}

    # MongoDB C++ driver
    /opt/mongo-cxx-driver/include/mongocxx/v_noabi
    /opt/mongo-cxx-driver/include/bsoncxx/v_noabi
)

add_executable(robot_motion_test_app
    src/main.cpp
    src/main_window.cpp
    src/dialogs.cpp
    src/graph_widget.cpp
    src/ros_client.cpp
    src/test_execution_controller.cpp

    include/robot_motion_test_app/main_window.h
    include/robot_motion_test_app/dialogs.h
    include/robot_motion_test_app/graph_widget.h
    include/robot_motion_test_app/ros_client.h
    include/robot_motion_test_app/test_execution_controller.h

    ui/main_window.ui
    ui/add_test_case.ui
    ui/load_prompt.ui
)

target_link_libraries(robot_motion_test_app
    ${catkin_LIBRARIES}
    Qt5::Widgets
    robot_motion_test_data
)