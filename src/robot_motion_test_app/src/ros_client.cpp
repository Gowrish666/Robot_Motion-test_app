#include "robot_motion_test_app/ros_client.h"

#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/Config.h>

#include <mission_msgs/ExecuteMissionActionGoal.h>
#include <mission_msgs/ExecuteMissionActionResult.h>


RosClient::RosClient(QObject* parent)
    : QObject(parent)
{
   

    graph_subscriber_ =
        node_handle_.subscribe(
            "/graph",
            1,
            &RosClient::graphCallback,
            this);


   

    telemetry_subscriber_ =
        node_handle_.subscribe(
            "/robot_motion_test/telemetry",
            20,
            &RosClient::telemetryCallback,
            this);

    
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


   

    execute_mission_goal_publisher_ =
        node_handle_.advertise<
            mission_msgs::ExecuteMissionActionGoal>(
                "/execute_mission/goal",
                1);


    execute_mission_result_subscriber_ =
        node_handle_.subscribe(
            "/execute_mission/result",
            10,
            &RosClient::executeMissionResultCallback,
            this);


    // ExecuteMission status subscriber.
    // The mission executor reports mission completion on this topic.
    execute_mission_status_subscriber_ =
        node_handle_.subscribe(
            "/execute_mission/status",
            10,
            &RosClient::executeMissionStatusCallback,
            this);


    

    motion_config_client_ =
        node_handle_.serviceClient<
            dynamic_reconfigure::Reconfigure>(
                "/move_base/AnscerLocalPlanner/set_parameters");


    controller_config_client_ =
        node_handle_.serviceClient<
            dynamic_reconfigure::Reconfigure>(
                "/controller_node/set_parameters");


   

    sto_publisher_ =
        node_handle_.advertise<std_msgs::Bool>(
            "/brake_activation/sto",
            1);


    safe_stop_publisher_ =
        node_handle_.advertise<std_msgs::Bool>(
            "brake_activation/safe_stop",
            1);


   

    mission_cancel_publisher_ =
        node_handle_.advertise<actionlib_msgs::GoalID>(
            "/execute_mission/cancel",
            1);


   

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



QStringList RosClient::waypointIds() const
{
    return waypoint_ids_;
}




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


    waypoint_name_to_id_ =
        new_waypoint_name_to_id;


    if (new_waypoints != waypoint_ids_)
    {
        waypoint_ids_ =
            new_waypoints;

        emit waypointsUpdated();
    }
}




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




void RosClient::executeMissionResultCallback(
    const mission_msgs::ExecuteMissionActionResult::ConstPtr& msg)
{
    if (msg->status.status ==
        actionlib_msgs::GoalStatus::SUCCEEDED)
    {
        const QString goalId =
            QString::fromStdString(
                msg->status.goal_id.id);

        /*
         * If a goal ID is available, make sure this result belongs
         * to the currently active goal.
         *
         * This prevents an old mission result from being interpreted
         * as completion of a newer return mission.
         */
        if (!goalId.isEmpty() &&
            !active_goal_id_.isEmpty() &&
            goalId != active_goal_id_)
        {
            ROS_INFO(
                "Ignoring ExecuteMission result for old goal: %s",
                goalId.toStdString().c_str());

            return;
        }

        if (!goalId.isEmpty())
        {
            if (completed_goal_id_ == goalId)
            {
                return;
            }

            completed_goal_id_ = goalId;
        }

        ROS_INFO(
            "==================================================");

        ROS_INFO(
            "ExecuteMission goal reached successfully.");

        if (!goalId.isEmpty())
        {
            ROS_INFO(
                "  goal_id = %s",
                goalId.toStdString().c_str());
        }

        ROS_INFO(
            "Waypoint mission completed.");

        ROS_INFO(
            "==================================================");


        emit goalReached();
    }
}




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
         * Ignore status messages belonging to an older mission.
         *
         * This is important because /execute_mission/status can
         * continue publishing SUCCEEDED for a completed goal.
         */
        if (goalId != active_goal_id_)
        {
            continue;
        }

        if (status.status ==
            actionlib_msgs::GoalStatus::SUCCEEDED)
        {
            /*
             * The same SUCCEEDED status can be published multiple
             * times. Only notify the controller once for this goal.
             */
            if (completed_goal_id_ == goalId)
            {
                return;
            }

            completed_goal_id_ = goalId;

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

bool RosClient::sendGoalByNodeId(int nodeId)
{
    if (nodeId < 0)
    {
        ROS_ERROR(
            "Cannot send mission goal: "
            "invalid node ID.");

        return false;
    }

    const ros::Time now =
        ros::Time::now();

    const uint64_t missionIdMilliseconds =
        static_cast<uint64_t>(
            now.toSec() * 1000.0);

    const std::string missionId =
        std::to_string(missionIdMilliseconds);

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
        std::to_string(now.toSec()) +
        "_" +
        missionId;

    /*
     * Store the exact goal ID that was sent to mission_executor.
     *
     * /execute_mission/status will be filtered using this ID.
     */
    active_goal_id_ =
        QString::fromStdString(
            message.goal_id.id);

    completed_goal_id_.clear();

    message.goal.xml_content =
        xmlContent;

    message.goal.mission_id =
        missionId;

    message.goal.mission_name =
        missionId;

    message.goal.execution_mode =
        2;

    ROS_INFO(
        "==================================================");

    ROS_INFO(
        "Sending ExecuteMission goal using manual node ID");

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


    if (!waypoint_name_to_id_.contains(
            goalWaypoint))
    {
        ROS_ERROR(
            "Waypoint '%s' was not found in the graph.",
            goalWaypoint.toStdString().c_str());

        return false;
    }


    const int nodeId =
        waypoint_name_to_id_.value(
            goalWaypoint);


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

    /*
     * Store the exact goal ID that was sent to mission_executor.
     *
     * This allows /execute_mission/status to identify completion
     * of the currently active forward/return mission.
     */
    active_goal_id_ =
        QString::fromStdString(
            message.goal_id.id);

    completed_goal_id_.clear();


    message.goal.xml_content =
        xmlContent;


    message.goal.mission_id =
        missionId;


    message.goal.mission_name =
        missionId;


    message.goal.execution_mode =
        2;


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




void RosClient::triggerSto()
{
    std_msgs::Bool message;

    message.data = true;

    sto_publisher_.publish(
        message);

    ROS_INFO(
        "STO activation published on /brake_activation/sto");
}




void RosClient::releaseSto()
{
    std_msgs::Bool message;

    message.data = false;

    sto_publisher_.publish(
        message);

    ROS_INFO(
        "STO release published on /brake_activation/sto");
}




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




void RosClient::stopRobot()
{
    actionlib_msgs::GoalID cancel_message;

    mission_cancel_publisher_.publish(
        cancel_message);

    ROS_INFO(
        "ExecuteMission cancellation published for "
        "normal simulated stop.");
}