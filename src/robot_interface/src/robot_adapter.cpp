#include "robot_interface/robot_adapter.h"

namespace robot_interface
{

RobotAdapter::RobotAdapter(
    ros::NodeHandle& node_handle,
    const std::string& graph_topic,
    const std::string& goal_topic,
    const std::string& pause_service,
    const std::string& stop_service)
    : node_handle_(node_handle),
      has_graph_(false)
{
    graph_subscriber_ =
        node_handle_.subscribe(
            graph_topic,
            1,
            &RobotAdapter::graphCallback,
            this);

    goal_publisher_ =
        node_handle_.advertise<geometry_msgs::PoseStamped>(
            goal_topic,
            1);

    pause_client_ =
        node_handle_.serviceClient<std_srvs::SetBool>(
            pause_service);

    stop_client_ =
        node_handle_.serviceClient<std_srvs::SetBool>(
            stop_service);

    ROS_INFO("RobotAdapter started.");
    ROS_INFO("Graph topic: %s", graph_topic.c_str());
    ROS_INFO("Goal topic: %s", goal_topic.c_str());
    ROS_INFO("Pause service: %s", pause_service.c_str());
    ROS_INFO("Stop service: %s", stop_service.c_str());
}

void RobotAdapter::graphCallback(
    const graph_msgs::Graph::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);

    waypoints_.clear();
    waypoints_.reserve(msg->vertices.size());

    graph_frame_id_ = msg->header.frame_id;

    for (const auto& vertex : msg->vertices)
    {
        Waypoint waypoint;
        waypoint.id = vertex.id;
        waypoint.name = vertex.name;
        waypoint.alias = vertex.alias;
        waypoint.pose = vertex.pose;

        waypoints_.push_back(waypoint);
    }

    has_graph_ = !waypoints_.empty();

    ROS_INFO(
        "Graph received: %zu waypoints, frame='%s'",
        waypoints_.size(),
        graph_frame_id_.c_str());
}

bool RobotAdapter::moveToWaypoint(
    const std::string& waypoint_name)
{
    Waypoint waypoint;

    if (!getWaypoint(waypoint_name, waypoint))
        return false;

    geometry_msgs::PoseStamped goal;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        goal.header.frame_id =
            graph_frame_id_.empty() ? "map" : graph_frame_id_;
    }

    goal.header.stamp = ros::Time::now();
    goal.pose = waypoint.pose;

    goal_publisher_.publish(goal);

    ROS_INFO(
        "Goal sent: waypoint=%s id=%u",
        waypoint.name.c_str(),
        waypoint.id);

    return true;
}

bool RobotAdapter::moveToWaypoint(
    std::uint32_t waypoint_id)
{
    Waypoint waypoint;

    if (!getWaypoint(waypoint_id, waypoint))
        return false;

    return moveToWaypoint(waypoint.name);
}

bool RobotAdapter::pauseMission(bool pause)
{
    if (!pause_client_.waitForExistence(ros::Duration(2.0)))
        return false;

    std_srvs::SetBool service;
    service.request.data = pause;

    if (!pause_client_.call(service))
        return false;

    return service.response.success;
}

bool RobotAdapter::stopRobot(bool stop)
{
    if (!stop_client_.waitForExistence(ros::Duration(2.0)))
        return false;

    std_srvs::SetBool service;
    service.request.data = stop;

    if (!stop_client_.call(service))
        return false;

    return service.response.success;
}

std::vector<Waypoint> RobotAdapter::waypoints() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return waypoints_;
}

bool RobotAdapter::hasWaypoint(
    const std::string& waypoint_name) const
{
    Waypoint waypoint;
    return getWaypoint(waypoint_name, waypoint);
}

bool RobotAdapter::hasWaypoint(
    std::uint32_t waypoint_id) const
{
    Waypoint waypoint;
    return getWaypoint(waypoint_id, waypoint);
}

bool RobotAdapter::getWaypoint(
    const std::string& waypoint_name,
    Waypoint& waypoint) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& candidate : waypoints_)
    {
        if (candidate.name == waypoint_name)
        {
            waypoint = candidate;
            return true;
        }
    }

    return false;
}

bool RobotAdapter::getWaypoint(
    std::uint32_t waypoint_id,
    Waypoint& waypoint) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto& candidate : waypoints_)
    {
        if (candidate.id == waypoint_id)
        {
            waypoint = candidate;
            return true;
        }
    }

    return false;
}

bool RobotAdapter::hasGraph() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return has_graph_;
}

}  // namespace robot_interface
