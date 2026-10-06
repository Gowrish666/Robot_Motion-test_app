#include "robot_motion_test_app/ros_client.h"

#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

#include <dynamic_reconfigure/Config.h>

#include <mission_msgs/ExecuteMissionActionGoal.h>
#include <mission_msgs/ExecuteMissionActionResult.h>

#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>


RosClient::RosClient(QObject* parent)
    : QObject(parent)
{
    // ============================================================
    // GRAPH
    // ============================================================

    graph_subscriber_ =
        node_handle_.subscribe(
            "/graph",
            1,
            &RosClient::graphCallback,
            this);


    // ============================================================
    // TELEMETRY
    // ============================================================

    telemetry_subscriber_ =
        node_handle_.subscribe(
            "/robot_motion_test/telemetry",
            20,
            &RosClient::telemetryCallback,
            this);


    // ============================================================
    // BRAKE FEEDBACK
    // ============================================================

    sto_feedback_subscriber_ =
        node_handle_.subscribe(
            "/brake_feedback/sto",
            10,
            &RosClient::stoFeedbackCallback,
            this);

    safe_stop_feedback_subscriber_ =
        node_handle_.subscribe(
            "/brake_feedback/safe_stop",
            10,
            &RosClient::safeStopFeedbackCallback,
            this);


    // ============================================================
    // EXECUTE MISSION
    // ============================================================

    execute_mission_goal_publisher_ =
        node_handle_.advertise<
            mission_msgs::ExecuteMissionActionGoal>(
                "/execute_mission/goal",
                1);


    /*
     * Keep the existing result subscriber.
     *
     * The real completion handling is now also done through
     * /execute_mission/status because that is the topic reporting
     * the actual mission state on the robot.
     */
    execute_mission_result_subscriber_ =
        node_handle_.subscribe(
            "/execute_mission/result",
            10,
            &RosClient::executeMissionResultCallback,
            this);


    /*
     * IMPORTANT:
     *
     * The robot reports ExecuteMission completion through
     *
     *     /execute_mission/status
     *
     * with actionlib_msgs/GoalStatusArray.
     *
     * This is required for the return-to-start mission to notify
     * TestExecutionController that the iteration has completed.
     */
    execute_mission_status_subscriber_ =
        node_handle_.subscribe(
            "/execute_mission/status",
            10,
            &RosClient::executeMissionStatusCallback,
            this);


    // ============================================================
    // DYNAMIC RECONFIGURE
    // ============================================================

    motion_config_client_ =
        node_handle_.serviceClient<
            dynamic_reconfigure::Reconfigure>(
                "/move_base/AnscerLocalPlanner/set_parameters");


    controller_config_client_ =
        node_handle_.serviceClient<
            dynamic_reconfigure::Reconfigure>(
                "/controller_node/set_parameters");


    // ============================================================
    // STO / SAFE-STOP
    // ============================================================

    sto_publisher_ =
        node_handle_.advertise<std_msgs::Bool>(
            "/brake_activation/sto",
            1);


    safe_stop_publisher_ =
        node_handle_.advertise<std_msgs::Bool>(
            "brake_activation/safe_stop",
            1);


    // ============================================================
    // EXECUTE MISSION CANCEL
    //
    // Used by the normal STOP button and Normal Braking.
    // STO / SAFE-STOP automatic activation does NOT call this.
    // ============================================================

    mission_cancel_publisher_ =
        node_handle_.advertise<actionlib_msgs::GoalID>(
            "/execute_mission/cancel",
            1);


    // ============================================================
    // NORMAL BRAKING STOP COMMAND
    // ============================================================

    normal_brake_stop_publisher_ =
        node_handle_.advertise<geometry_msgs::Twist>(
            "/cmd_vel",
            1);


    // ============================================================
    // ROS SPINNER
    // ============================================================

    spinner_ =
        std::make_unique<ros::AsyncSpinner>(1);

    spinner_->start();
}


RosClient::~RosClient()
{
    if (spinner_)
    {
        spinner_->stop();
    }
}


// ================================================================
// WAYPOINTS
// ================================================================

QStringList RosClient::waypointIds() const
{
    std::lock_guard<std::mutex> lock(
        graph_mutex_);

    return waypoint_ids_;
}


// ================================================================
// GRAPH CALLBACK
// ================================================================

void RosClient::graphCallback(
    const graph_msgs::Graph::ConstPtr& msg)
{
    QStringList new_waypoints;

    QHash<QString, int> new_waypoint_name_to_id;


    for (const auto& vertex : msg->vertices)
    {
        QString waypoint;


        if (!vertex.name.empty())
        {
            waypoint =
                QString::fromStdString(
                    vertex.name);
        }
        else if (!vertex.alias.empty())
        {
            waypoint =
                QString::fromStdString(
                    vertex.alias);
        }
        else
        {
            waypoint =
                QString::number(
                    vertex.id);
        }


        if (waypoint.isEmpty())
        {
            continue;
        }


        if (!new_waypoints.contains(waypoint))
        {
            new_waypoints.append(waypoint);

            new_waypoint_name_to_id.insert(
                waypoint,
                static_cast<int>(vertex.id));
        }
    }


    bool waypoints_changed = false;

    {
        std::lock_guard<std::mutex> lock(
            graph_mutex_);

        latest_graph_ = *msg;

        waypoint_name_to_id_ =
            new_waypoint_name_to_id;

        waypoints_changed =
            (new_waypoints != waypoint_ids_);

        if (waypoints_changed)
        {
            waypoint_ids_ =
                new_waypoints;
        }
    }

    if (waypoints_changed)
    {
        emit waypointsUpdated();
    }
}


// ================================================================
// ROUTE DISTANCE
// ================================================================

double RosClient::routeDistance(
    const QString& startWaypoint,
    const QString& goalWaypoint) const
{
    int startNodeId = -1;
    int goalNodeId = -1;

    {
        std::lock_guard<std::mutex> lock(
            graph_mutex_);

        if (!waypoint_name_to_id_.contains(
                startWaypoint) ||
            !waypoint_name_to_id_.contains(
                goalWaypoint))
        {
            return -1.0;
        }

        startNodeId =
            waypoint_name_to_id_.value(
                startWaypoint);

        goalNodeId =
            waypoint_name_to_id_.value(
                goalWaypoint);
    }

    return routeDistanceByNodeId(
        startNodeId,
        goalNodeId);
}


double RosClient::routeDistanceByNodeId(
    int startNodeId,
    int goalNodeId) const
{
    if (startNodeId < 0 ||
        goalNodeId < 0)
    {
        return -1.0;
    }

    graph_msgs::Graph graph_copy;

    {
        std::lock_guard<std::mutex> lock(
            graph_mutex_);

        graph_copy = latest_graph_;
    }

    return calculateRouteDistance(
        graph_copy,
        startNodeId,
        goalNodeId);
}


double RosClient::calculateRouteDistance(
    const graph_msgs::Graph& graph,
    int startNodeId,
    int goalNodeId)
{
    if (startNodeId == goalNodeId)
    {
        return 0.0;
    }

    using NodeId = uint32_t;
    using Neighbor =
        std::pair<NodeId, double>;

    std::unordered_map<
        NodeId,
        std::vector<Neighbor>>
        adjacency;

    bool start_exists = false;
    bool goal_exists = false;

    for (const auto& vertex : graph.vertices)
    {
        if (static_cast<int>(vertex.id) ==
            startNodeId)
        {
            start_exists = true;
        }

        if (static_cast<int>(vertex.id) ==
            goalNodeId)
        {
            goal_exists = true;
        }

        adjacency.emplace(
            vertex.id,
            std::vector<Neighbor>());
    }

    if (!start_exists || !goal_exists)
    {
        return -1.0;
    }

    auto add_edge =
        [&adjacency](
            NodeId from,
            NodeId to,
            double length)
        {
            if (!std::isfinite(length) ||
                length <= 0.0)
            {
                return;
            }

            adjacency[from].push_back(
                std::make_pair(to, length));
        };

    for (const auto& edge : graph.edges)
    {
        const NodeId source =
            edge.source_vertex_id;

        const NodeId target =
            edge.target_vertex_id;

        const double length =
            edge.length;

        switch (edge.edge_direction_type)
        {
        case graph_msgs::Edge::FORWARD:
            add_edge(
                source,
                target,
                length);

            if (edge.bidirectional)
            {
                add_edge(
                    target,
                    source,
                    length);
            }
            break;

        case graph_msgs::Edge::REVERSE:
            add_edge(
                target,
                source,
                length);

            if (edge.bidirectional)
            {
                add_edge(
                    source,
                    target,
                    length);
            }
            break;

        case graph_msgs::Edge::BIDIRECTIONAL:
            add_edge(
                source,
                target,
                length);

            add_edge(
                target,
                source,
                length);
            break;

        default:
            if (edge.bidirectional)
            {
                add_edge(
                    source,
                    target,
                    length);

                add_edge(
                    target,
                    source,
                    length);
            }
            break;
        }
    }

    using QueueItem =
        std::pair<double, NodeId>;

    const double infinity =
        std::numeric_limits<double>::infinity();

    std::unordered_map<NodeId, double> distance;

    for (const auto& vertex : graph.vertices)
    {
        distance[vertex.id] = infinity;
    }

    const NodeId start =
        static_cast<NodeId>(startNodeId);

    const NodeId goal =
        static_cast<NodeId>(goalNodeId);

    std::priority_queue<
        QueueItem,
        std::vector<QueueItem>,
        std::greater<QueueItem>>
        queue;

    distance[start] = 0.0;
    queue.push(
        std::make_pair(0.0, start));

    while (!queue.empty())
    {
        const double currentDistance =
            queue.top().first;

        const NodeId currentNode =
            queue.top().second;

        queue.pop();

        if (currentDistance >
            distance[currentNode])
        {
            continue;
        }

        if (currentNode == goal)
        {
            return currentDistance;
        }

        const auto adjacencyIt =
            adjacency.find(currentNode);

        if (adjacencyIt == adjacency.end())
        {
            continue;
        }

        for (const auto& neighbor :
             adjacencyIt->second)
        {
            const NodeId nextNode =
                neighbor.first;

            const double newDistance =
                currentDistance +
                neighbor.second;

            if (newDistance <
                distance[nextNode])
            {
                distance[nextNode] =
                    newDistance;

                queue.push(
                    std::make_pair(
                        newDistance,
                        nextNode));
            }
        }
    }

    return -1.0;
}


// ================================================================
// TELEMETRY CALLBACK
// ================================================================

void RosClient::telemetryCallback(
    const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 4)
    {
        ROS_WARN_THROTTLE(
            5.0,
            "Telemetry message has fewer than 4 values.");

        return;
    }


    const double time =
        msg->data[0];

    const double velocity =
        msg->data[1];

    const double distance =
        msg->data[2];


    emit velocityUpdated(
        velocity,
        distance,
        time);
}


// ================================================================
// BRAKE FEEDBACK
// ================================================================

void RosClient::stoFeedbackCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
    emit stoFeedbackUpdated(msg->data);
}


void RosClient::safeStopFeedbackCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
    emit safeStopFeedbackUpdated(msg->data);
}


// ================================================================
// EXECUTE MISSION RESULT CALLBACK
// ================================================================

void RosClient::executeMissionResultCallback(
    const mission_msgs::ExecuteMissionActionResult::ConstPtr& msg)
{
    /*
     * Keep this callback for compatibility with the existing
     * ExecuteMission result topic.
     *
     * The actual robot completion is handled through
     * /execute_mission/status.
     */
    if (msg->status.status ==
        actionlib_msgs::GoalStatus::SUCCEEDED)
    {
        ROS_INFO(
            "ExecuteMission result reports SUCCEEDED.");

        /*
         * Do not emit goalReached() here.
         *
         * /execute_mission/status is the authoritative completion
         * source used by this application.
         */
    }
}


// ================================================================
// EXECUTE MISSION STATUS CALLBACK
// ================================================================

void RosClient::executeMissionStatusCallback(
    const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
    for (const auto& status : msg->status_list)
    {
        const QString goalId =
            QString::fromStdString(
                status.goal_id.id);


        if (goalId.isEmpty())
        {
            continue;
        }


        /*
         * Ignore missions that were started before the currently
         * active mission.
         */
        if (goalId != active_goal_id_)
        {
            continue;
        }


        ROS_INFO(
            "ExecuteMission status: status=%d goal_id='%s'",
            status.status,
            goalId.toStdString().c_str());


        // ========================================================
        // SUCCESS
        // ========================================================

        if (status.status ==
            actionlib_msgs::GoalStatus::SUCCEEDED)
        {
            /*
             * Prevent the same mission from generating
             * goalReached() more than once.
             */
            if (completed_goal_id_ == goalId)
            {
                return;
            }


            completed_goal_id_ =
                goalId;


            ROS_INFO(
                "==================================================");

            ROS_INFO(
                "ExecuteMission goal reached successfully "
                "via /execute_mission/status.");

            ROS_INFO(
                "  goal_id = %s",
                goalId.toStdString().c_str());

            ROS_INFO(
                "Waypoint mission completed.");

            ROS_INFO(
                "==================================================");


            emit goalReached();

            return;
        }
    }
}


// ================================================================
// CONFIGURE MOTION
// ================================================================

bool RosClient::configureMotion(
    double maxVelocity,
    double acceleration)
{
    if (maxVelocity <= 0.0)
    {
        ROS_ERROR(
            "Invalid test-case maximum velocity: %.4f m/s",
            maxVelocity);

        return false;
    }


    if (acceleration <= 0.0)
    {
        ROS_ERROR(
            "Invalid test-case acceleration: %.4f m/s^2",
            acceleration);

        return false;
    }


    // ============================================================
    // PLANNER CONFIGURATION
    // ============================================================

    if (!motion_config_client_.waitForExistence(
            ros::Duration(1.0)))
    {
        ROS_ERROR(
            "Planner dynamic-reconfigure service is unavailable: "
            "/move_base/AnscerLocalPlanner/set_parameters");

        return false;
    }


    dynamic_reconfigure::Reconfigure planner_service;


    dynamic_reconfigure::DoubleParameter planner_max_vel;

    planner_max_vel.name =
        "max_vel";

    planner_max_vel.value =
        maxVelocity;


    dynamic_reconfigure::DoubleParameter planner_max_acc;

    planner_max_acc.name =
        "max_acc";

    planner_max_acc.value =
        acceleration;


    dynamic_reconfigure::DoubleParameter planner_max_decel;

    planner_max_decel.name =
        "max_decel";

    planner_max_decel.value =
        acceleration;


    planner_service.request.config.doubles.push_back(
        planner_max_vel);

    planner_service.request.config.doubles.push_back(
        planner_max_acc);

    planner_service.request.config.doubles.push_back(
        planner_max_decel);


    if (!motion_config_client_.call(
            planner_service))
    {
        ROS_ERROR(
            "Failed to configure AnscerLocalPlanner.");

        return false;
    }


    ROS_INFO(
        "AnscerLocalPlanner configured: "
        "max_vel=%.4f max_acc=%.4f max_decel=%.4f",
        maxVelocity,
        acceleration,
        acceleration);


    // ============================================================
    // CONTROLLER CONFIGURATION
    // ============================================================

    if (!controller_config_client_.waitForExistence(
            ros::Duration(1.0)))
    {
        ROS_ERROR(
            "Controller dynamic-reconfigure service is unavailable: "
            "/controller_node/set_parameters");

        return false;
    }


    dynamic_reconfigure::Reconfigure controller_service;


    dynamic_reconfigure::DoubleParameter
        controller_max_linear_vel;

    controller_max_linear_vel.name =
        "max_linear_velocity";

    controller_max_linear_vel.value =
        maxVelocity;


    dynamic_reconfigure::DoubleParameter
        controller_curve_max_vel;

    controller_curve_max_vel.name =
        "max_curve_linear_velocity";

    controller_curve_max_vel.value =
        maxVelocity;


    dynamic_reconfigure::DoubleParameter
        controller_max_acc;

    controller_max_acc.name =
        "max_acceleration";

    controller_max_acc.value =
        acceleration;


    controller_service.request.config.doubles.push_back(
        controller_max_linear_vel);

    controller_service.request.config.doubles.push_back(
        controller_curve_max_vel);

    controller_service.request.config.doubles.push_back(
        controller_max_acc);


    if (!controller_config_client_.call(
            controller_service))
    {
        ROS_ERROR(
            "Failed to configure controller motion parameters.");

        return false;
    }


    ROS_INFO(
        "==================================================");

    ROS_INFO(
        "TEST CASE MOTION CONFIGURATION");

    ROS_INFO(
        "  test max velocity              = %.4f m/s",
        maxVelocity);

    ROS_INFO(
        "  test acceleration              = %.4f m/s^2",
        acceleration);

    ROS_INFO(
        "  planner max_vel                = %.4f",
        maxVelocity);

    ROS_INFO(
        "  controller max_linear_velocity = %.4f",
        maxVelocity);

    ROS_INFO(
        "  controller max_curve_velocity  = %.4f",
        maxVelocity);

    ROS_INFO(
        "  controller max_acceleration    = %.4f",
        acceleration);

    ROS_INFO(
        "==================================================");


    return true;
}


// ================================================================
// SEND GOAL
// ================================================================

bool RosClient::sendGoal(
    const QString& startWaypoint,
    const QString& goalWaypoint)
{
    Q_UNUSED(startWaypoint);


    if (goalWaypoint.isEmpty())
    {
        ROS_ERROR(
            "Cannot send mission goal: "
            "goal waypoint is empty.");

        return false;
    }


    int nodeId = -1;

    {
        std::lock_guard<std::mutex> lock(
            graph_mutex_);

        if (!waypoint_name_to_id_.contains(
                goalWaypoint))
        {
            ROS_ERROR(
                "Waypoint '%s' was not found in the graph.",
                goalWaypoint.toStdString().c_str());

            return false;
        }

        nodeId =
            waypoint_name_to_id_.value(
                goalWaypoint);
    }


    const ros::Time now =
        ros::Time::now();


    const uint64_t missionIdMilliseconds =
        static_cast<uint64_t>(
            now.toSec() * 1000.0);


    const std::string missionId =
        std::to_string(
            missionIdMilliseconds);


    const std::string xmlContent =
        "<?xml version=\"1.0\"?>\n"
        "<root main_tree_to_execute=\"BehaviorTree\">\n"
        "    <BehaviorTree ID=\"BehaviorTree\">\n"
        "        <Sequence name=\"1\">"
        "<Action ID=\"MoveToNode\" node_id_list=\"" +
        std::to_string(nodeId) +
        "\" name=\"MoveToNode_1_null\"/>"
        "</Sequence>\n"
        "    </BehaviorTree>\n"
        "</root>";


    mission_msgs::ExecuteMissionActionGoal message;


    message.header.stamp =
        now;

    message.goal_id.stamp =
        now;

    message.goal_id.id =
        "goal_" +
        std::to_string(
            now.toSec()) +
        "_" +
        missionId;

    message.goal.xml_content =
        xmlContent;

    message.goal.mission_id =
        missionId;

    message.goal.mission_name =
        missionId;

    message.goal.execution_mode =
        2;


    /*
     * IMPORTANT:
     *
     * Store this exact goal ID before publishing.
     *
     * The status callback uses this ID to identify the current
     * forward or return mission.
     */
    active_goal_id_ =
        QString::fromStdString(
            message.goal_id.id);

    completed_goal_id_.clear();


    ROS_INFO(
        "==================================================");

    ROS_INFO(
        "Sending ExecuteMission goal");

    ROS_INFO(
        "  waypoint   = %s",
        goalWaypoint.toStdString().c_str());

    ROS_INFO(
        "  node_id    = %d",
        nodeId);

    ROS_INFO(
        "  mission_id = %s",
        missionId.c_str());

    ROS_INFO(
        "  goal_id    = %s",
        message.goal_id.id.c_str());

    ROS_INFO(
        "  execution_mode = 2");

    ROS_INFO(
        "  XML:");

    ROS_INFO(
        "%s",
        xmlContent.c_str());

    ROS_INFO(
        "==================================================");


    execute_mission_goal_publisher_.publish(
        message);


    return true;
}


// ================================================================
// SEND GOAL BY NODE ID
// ================================================================

bool RosClient::sendGoalByNodeId(
    int nodeId)
{
    if (nodeId < 0)
    {
        ROS_ERROR(
            "Cannot send mission goal: "
            "invalid node ID: %d",
            nodeId);

        return false;
    }


    const ros::Time now =
        ros::Time::now();


    const uint64_t missionIdMilliseconds =
        static_cast<uint64_t>(
            now.toSec() * 1000.0);


    const std::string missionId =
        std::to_string(
            missionIdMilliseconds);


    const std::string xmlContent =
        "<?xml version=\"1.0\"?>\n"
        "<root main_tree_to_execute=\"BehaviorTree\">\n"
        "    <BehaviorTree ID=\"BehaviorTree\">\n"
        "        <Sequence name=\"1\">"
        "<Action ID=\"MoveToNode\" node_id_list=\"" +
        std::to_string(nodeId) +
        "\" name=\"MoveToNode_1_null\"/>"
        "</Sequence>\n"
        "    </BehaviorTree>\n"
        "</root>";


    mission_msgs::ExecuteMissionActionGoal message;


    message.header.stamp =
        now;

    message.goal_id.stamp =
        now;

    message.goal_id.id =
        "goal_" +
        std::to_string(
            now.toSec()) +
        "_" +
        missionId;

    message.goal.xml_content =
        xmlContent;

    message.goal.mission_id =
        missionId;

    message.goal.mission_name =
        missionId;

    message.goal.execution_mode =
        2;


    /*
     * Store this goal ID as the active mission.
     */
    active_goal_id_ =
        QString::fromStdString(
            message.goal_id.id);

    completed_goal_id_.clear();


    ROS_INFO(
        "==================================================");

    ROS_INFO(
        "Sending ExecuteMission goal by node ID");

    ROS_INFO(
        "  node_id    = %d",
        nodeId);

    ROS_INFO(
        "  mission_id = %s",
        missionId.c_str());

    ROS_INFO(
        "  goal_id    = %s",
        message.goal_id.id.c_str());

    ROS_INFO(
        "  execution_mode = 2");

    ROS_INFO(
        "  XML:");

    ROS_INFO(
        "%s",
        xmlContent.c_str());

    ROS_INFO(
        "==================================================");


    execute_mission_goal_publisher_.publish(
        message);


    return true;
}


// ================================================================
// STO
// ================================================================

void RosClient::triggerSto()
{
    std_msgs::Bool message;

    message.data = true;

    sto_publisher_.publish(
        message);

    ROS_INFO(
        "STO activation published on /brake_activation/sto");
}


// ================================================================
// RELEASE STO
// ================================================================

void RosClient::releaseSto()
{
    std_msgs::Bool message;

    message.data = false;

    sto_publisher_.publish(
        message);

    ROS_INFO(
        "STO release published on /brake_activation/sto");
}


// ================================================================
// SAFE-STOP
// ================================================================

void RosClient::triggerSafeStop()
{
    std_msgs::Bool message;

    message.data = true;

    safe_stop_publisher_.publish(
        message);

    ROS_INFO(
        "Safe-Stop activation published on "
        "brake_activation/safe_stop");
}


// ================================================================
// RELEASE SAFE-STOP
// ================================================================

void RosClient::releaseSafeStop()
{
    std_msgs::Bool message;

    message.data = false;

    safe_stop_publisher_.publish(
        message);

    ROS_INFO(
        "Safe-Stop release published on "
        "brake_activation/safe_stop");
}


// ================================================================
// NORMAL STOP
// ================================================================

void RosClient::stopRobot()
{
    actionlib_msgs::GoalID cancel_message;

    mission_cancel_publisher_.publish(
        cancel_message);

    ROS_INFO(
        "ExecuteMission cancellation published for "
        "normal simulated stop.");
}


// ================================================================
// NORMAL BRAKING STOP
// ================================================================

void RosClient::publishNormalBrakeStop()
{
    geometry_msgs::Twist stop_message;

    stop_message.linear.x = 0.0;
    stop_message.linear.y = 0.0;
    stop_message.linear.z = 0.0;

    stop_message.angular.x = 0.0;
    stop_message.angular.y = 0.0;
    stop_message.angular.z = 0.0;

    normal_brake_stop_publisher_.publish(
        stop_message);

    ROS_INFO(
        "Normal braking zero Twist published on /cmd_vel");
}