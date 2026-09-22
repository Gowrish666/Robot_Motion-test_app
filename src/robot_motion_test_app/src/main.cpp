#include <QApplication>

#include <ros/ros.h>

#include "robot_motion_test_app/main_window.h"

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "robot_motion_test_app");

    QApplication application(argc, argv);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    MainWindow window;
    window.show();

    const int result =
        application.exec();

    ros::shutdown();

    return result;
}