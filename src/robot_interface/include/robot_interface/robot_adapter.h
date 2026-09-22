#ifndef ROBOT_INTERFACE_ROBOT_ADAPTER_H
#define ROBOT_INTERFACE_ROBOT_ADAPTER_H

#include <geometry_msgs/PoseStamped.h>
#include <graph_msgs/Graph.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace robot_interface
{

struct Waypoint
{
    std::uint32_t id = 0;
    std::string name;
    std::string alias;
    geometry_msgs::Pose pose;
};

class RobotAdapter
{
public:
    RobotAdapter(
        ros::NodeHandle& node_handle,
        const std::string& graph_topic = "/graph",
        const std::string& goal_topic = "/move_base_simple/goal",
        const std::string& pause_service = "/pause_mission",
        const std::string& stop_service = "/stop_robot");

    bool moveToWaypoint(const std::string& waypoint_name);
    bool moveToWaypoint(std::uint32_t waypoint_id);

    bool pauseMission(bool pause);
    bool stopRobot(bool stop = true);

    std::vector<Waypoint> waypoints() const;

    bool hasWaypoint(const std::string& waypoint_name) const;
    bool hasWaypoint(std::uint32_t waypoint_id) const;

    bool getWaypoint(
        const std::string& waypoint_name,
        Waypoint& waypoint) const;

    bool getWaypoint(
        std::uint32_t waypoint_id,
        Waypoint& waypoint) const;

    bool hasGraph() const;

private:
    void graphCallback(
        const graph_msgs::Graph::ConstPtr& msg);

    ros::NodeHandle node_handle_;
    ros::Subscriber graph_subscriber_;
    ros::Publisher goal_publisher_;
    ros::ServiceClient pause_client_;
    ros::ServiceClient stop_client_;

    mutable std::mutex mutex_;

    std::vector<Waypoint> waypoints_;
    std::string graph_frame_id_;
    bool has_graph_;
};

}  // namespace robot_interface

#endif
