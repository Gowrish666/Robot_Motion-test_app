#ifndef ROBOT_MOTION_TEST_APP_TEST_EXECUTION_CONTROLLER_H
#define ROBOT_MOTION_TEST_APP_TEST_EXECUTION_CONTROLLER_H

#include <QObject>
#include <QVector>
#include <QPointF>
#include <QTimer>

#include <vector>

#include "robot_motion_test_data/models.h"
#include "robot_motion_test_app/ros_client.h"

class TestExecutionController : public QObject
{
    Q_OBJECT

public:
    explicit TestExecutionController(
        RosClient* rosClient,
        QObject* parent = nullptr);

    void setTestCases(
        const std::vector<robot_motion_test_data::TestCase>& testCases);

    void setWaypoints(
        const QString& startWaypoint,
        const QString& goalWaypoint);

    void start();
    void stop();
    void setManualWaypoints(int startNodeId, int goalNodeId);
void clearManualWaypoints();
    void loadAccepted();

    bool isRunning() const;

signals:
    void testStarted(
        const QString& name,
        int iteration);

    void testFinished(
        const QString& name);

    void executionStopped();

    void executionCompleted();

    void statusChanged(
        const QString& status);

    void velocityDataChanged(
        const QVector<QPointF>& data);

    void velocityDistanceDataChanged(
        const QVector<QPointF>& data);

    void accelerationDataChanged(
        const QVector<QPointF>& data);

    void loadRequired(
        double massKg);

    void brakeActivated(
        const QString& label,
        double time,
        double distance);

private slots:
    void telemetryUpdated(
        double velocity,
        double distance,
        double time);

    void goalReached();

    // Brake feedback slots
    void stoFeedbackUpdated(bool active);

    void safeStopFeedbackUpdated(bool active);

    void maxVelocityHoldFinished();

    void zeroVelocityHoldFinished();

private:
    enum class Phase
    {
        IDLE,
        MOVING_TO_GOAL,
        HOLDING_MAX_VELOCITY,
        BRAKING,
        HOLDING_ZERO_VELOCITY,
        RETURNING_TO_START
    };

    void startCurrentTest();
    void finishCurrentIteration();
    void clearCurrentData();
    void startMaximumVelocityHold();
    void activateBrakeAndStop();
    void releaseBrakeAndReturn();
    void startZeroVelocityHold();
    void startReturnToStart();

    RosClient* ros_client_;

    std::vector<robot_motion_test_data::TestCase> test_cases_;

    QString start_waypoint_;
    QString goal_waypoint_;
    bool manual_waypoint_mode_;
int manual_start_node_id_;
int manual_goal_node_id_;

    bool running_;

    int current_test_index_;
    int current_iteration_;

    bool waiting_for_load_;
    bool load_prompt_shown_;

    Phase phase_;

    bool braking_started_;
    bool brake_released_;
    bool return_motion_detected_;

    // Brake feedback state
    bool sto_feedback_active_;
    bool safe_stop_feedback_active_;

    double braking_start_distance_;
    double braking_distance_;

    double braking_start_time_;
    double braking_time_;

    double max_velocity_achieved_;

    double previous_velocity_;
    double previous_time_;

    double latest_distance_;
    double latest_time_;

    bool maximum_velocity_hold_started_;
    bool zero_velocity_hold_started_;

    QTimer max_velocity_hold_timer_;
    QTimer zero_velocity_hold_timer_;

    QVector<QPointF> velocity_time_data_;
    QVector<QPointF> velocity_distance_data_;
    QVector<QPointF> acceleration_time_data_;
};

#endif
