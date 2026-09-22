#ifndef ROBOT_MOTION_TEST_APP_ROS_CLIENT_H
#define ROBOT_MOTION_TEST_APP_ROS_CLIENT_H

#include <QObject>
#include <QString>
#include <QStringList>
#include <memory>

#include <ros/ros.h>

#include <graph_msgs/Graph.h>

#include <robot_motion_test_msgs/MoveToWaypoint.h>

#include <mission_msgs/ExecuteMissionActionGoal.h>
#include <mission_msgs/ExecuteMissionActionResult.h>

#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <QHash>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <dynamic_reconfigure/Reconfigure.h>

class RosClient : public QObject
{
    Q_OBJECT

public:
    explicit RosClient(
        QObject* parent = nullptr);

    ~RosClient();

    QStringList waypointIds() const;

    bool sendGoal(
        const QString& startWaypoint,
        const QString& goalWaypoint);

    bool sendGoalByNodeId(
        int nodeId);

    bool configureMotion(
        double maxVelocity,
        double acceleration);

    void triggerSto();

    void releaseSto();

    void triggerSafeStop();

    void releaseSafeStop();

    void stopRobot();

signals:

    void velocityUpdated(
        double velocity,
        double distance,
        double time);

    void waypointsUpdated();

    void goalReached();

    // Brake feedback signals
    void stoFeedbackUpdated(bool active);

    void safeStopFeedbackUpdated(bool active);

private:

    void graphCallback(
        const graph_msgs::Graph::ConstPtr& msg);

    void telemetryCallback(
        const std_msgs::Float64MultiArray::ConstPtr& msg);

    void goalReachedCallback(
        const std_msgs::Bool::ConstPtr& msg);

    // Brake feedback callbacks
    void stoFeedbackCallback(
        const std_msgs::Bool::ConstPtr& msg);

    void safeStopFeedbackCallback(
        const std_msgs::Bool::ConstPtr& msg);

    void executeMissionResultCallback(
        const mission_msgs::ExecuteMissionActionResult::ConstPtr& msg);

    void executeMissionStatusCallback(
        const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

    ros::NodeHandle node_handle_;

    ros::Subscriber graph_subscriber_;

    ros::Subscriber telemetry_subscriber_;

    // Brake feedback subscribers
    ros::Subscriber sto_feedback_subscriber_;

    ros::Subscriber safe_stop_feedback_subscriber_;

    ros::Subscriber goal_reached_subscriber_;

    ros::Subscriber execute_mission_result_subscriber_;

    ros::Subscriber execute_mission_status_subscriber_;

    ros::Publisher execute_mission_goal_publisher_;

    QHash<QString, int> waypoint_name_to_id_;

    QStringList waypoint_ids_;

    ros::ServiceClient motion_config_client_;

    ros::ServiceClient controller_config_client_;

    ros::Publisher sto_publisher_;

    ros::Publisher safe_stop_publisher_;

    ros::Publisher mission_cancel_publisher_;

    // ExecuteMission goal tracking
    QString active_goal_id_;

    QString completed_goal_id_;

    std::unique_ptr<ros::AsyncSpinner> spinner_;
};

#endif