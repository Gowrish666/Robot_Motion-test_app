#include "robot_interface/odom_processor.h"
#include "robot_interface/robot_adapter.h"
#include "robot_motion_test_msgs/MoveToWaypoint.h"
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

namespace
{

void printOdomData(
    const robot_interface::OdomProcessor& odom_processor)
{
    ROS_INFO(
        "Time: %.3f s | Velocity: %.3f m/s | "
        "Distance: %.3f m | Acceleration: %.3f m/s^2",
        odom_processor.currentTime(),
        odom_processor.currentVelocity(),
        odom_processor.currentDistance(),
        odom_processor.currentAcceleration());
}

}  // namespace

namespace robot_interface
{

class RobotInterfaceNode
{
public:

    RobotInterfaceNode()
        : private_node_handle_("~"),
          odom_processor_(0.001, 0.2),
          robot_adapter_(node_handle_)
    {
        private_node_handle_.param(
            "odom_topic",
            odom_topic_,
            std::string("/odom"));

        private_node_handle_.param(
            "odom_min_dt",
            min_dt_,
            0.001);

        private_node_handle_.param(
            "odom_max_dt",
            max_dt_,
            0.2);

        odom_processor_ =
            OdomProcessor(
                min_dt_,
                max_dt_);

        odom_subscriber_ =
            node_handle_.subscribe(
                odom_topic_,
                100,
                &RobotInterfaceNode::odomCallback,
                this);

        telemetry_publisher_ =
            node_handle_.advertise<std_msgs::Float64MultiArray>(
                "/robot_motion_test/telemetry",
                10);

        goal_reached_publisher_ =
        node_handle_.advertise<std_msgs::Bool>(
        "/robot_motion_test/goal_reached",
        10);

        move_base_result_subscriber_ =
        node_handle_.subscribe(
        "/move_base/result",
        10,
        &RobotInterfaceNode::moveBaseResultCallback,
        this);

        move_to_waypoint_service_ =
            node_handle_.advertiseService(
                "/move_to_waypoint",
                &RobotInterfaceNode::moveToWaypointCallback,
                this);

        ROS_INFO("Robot Interface Node started.");
        ROS_INFO("Subscribing to: %s", odom_topic_.c_str());
        ROS_INFO("Publishing telemetry to: /robot_motion_test/telemetry");
        ROS_INFO("odom_min_dt: %.6f", min_dt_);
        ROS_INFO("odom_max_dt: %.6f", max_dt_);
    }

    void spin()
    {
        ros::Rate loop_rate(10.0);

        while (ros::ok())
        {
            if (odom_processor_.hasData())
            {
                printOdomData(odom_processor_);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:

    void odomCallback(
        const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_processor_.processOdom(msg);

        if (!odom_processor_.hasData())
        {
            return;
        }

        std_msgs::Float64MultiArray telemetry;

        telemetry.data.resize(4);

        telemetry.data[0] =
            odom_processor_.currentTime();

        telemetry.data[1] =
            odom_processor_.currentVelocity();

        telemetry.data[2] =
            odom_processor_.currentDistance();

        telemetry.data[3] =
            odom_processor_.currentAcceleration();

        telemetry_publisher_.publish(telemetry);
    }

    bool moveToWaypointCallback(
        robot_motion_test_msgs::MoveToWaypoint::Request& request,
        robot_motion_test_msgs::MoveToWaypoint::Response& response)
    {
        if (request.waypoint_name.empty())
        {
            response.success = false;
            response.message = "Waypoint name is empty.";
            return true;
        }

        const bool success =
            robot_adapter_.moveToWaypoint(request.waypoint_name);

        response.success = success;
        response.message =
            success
                ? "Goal sent to waypoint: " + request.waypoint_name
                : "Waypoint not found: " + request.waypoint_name;

        return true;
    }

    void moveBaseResultCallback(
    const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    if (msg->status.status == 3)
    {
        std_msgs::Bool reached;
        reached.data = true;

        goal_reached_publisher_.publish(
            reached);

        ROS_INFO(
            "Robot reached navigation goal.");
    }
}

    ros::NodeHandle node_handle_;
    ros::NodeHandle private_node_handle_;

    ros::Subscriber odom_subscriber_;
    ros::Publisher telemetry_publisher_;
    
    ros::Subscriber move_base_result_subscriber_;
    ros::Publisher goal_reached_publisher_;

    ros::ServiceServer move_to_waypoint_service_;

    OdomProcessor odom_processor_;
    RobotAdapter robot_adapter_;

    std::string odom_topic_;

    double min_dt_;
    double max_dt_;
};

}  // namespace robot_interface

int main(int argc, char** argv)
{
    ros::init(
        argc,
        argv,
        "robot_interface_node");

    robot_interface::RobotInterfaceNode node;

    node.spin();

    return 0;
}
