#include "robot_motion_test_app/test_execution_controller.h"
#include "robot_motion_test_data/data_access_layer.h"

#include <algorithm>
#include <cmath>

#include <QThread>


TestExecutionController::TestExecutionController(
    RosClient* rosClient,
    QObject* parent)
    : QObject(parent),
      ros_client_(rosClient),
      running_(false),
      current_test_index_(0),
      current_iteration_(0),
      waiting_for_load_(false),
      load_prompt_shown_(false),
      phase_(Phase::IDLE),
      braking_started_(false),
      brake_released_(false),
      return_motion_detected_(false),
      sto_feedback_active_(false),
      safe_stop_feedback_active_(false),
      manual_waypoint_mode_(false),
      manual_start_node_id_(-1),
      manual_goal_node_id_(-1),
      braking_start_distance_(0.0),
      braking_distance_(0.0),
      braking_start_time_(0.0),
      braking_time_(0.0),
      max_velocity_achieved_(0.0),
      previous_velocity_(0.0),
      previous_time_(0.0),
      latest_distance_(0.0),
      latest_time_(0.0),
      maximum_velocity_hold_started_(false),
      zero_velocity_hold_started_(false)
{
    connect(
        ros_client_,
        &RosClient::velocityUpdated,
        this,
        &TestExecutionController::telemetryUpdated);


    connect(
        ros_client_,
        &RosClient::goalReached,
        this,
        &TestExecutionController::goalReached);


    connect(
        ros_client_,
        &RosClient::stoFeedbackUpdated,
        this,
        &TestExecutionController::stoFeedbackUpdated);


    connect(
        ros_client_,
        &RosClient::safeStopFeedbackUpdated,
        this,
        &TestExecutionController::safeStopFeedbackUpdated);


    max_velocity_hold_timer_.setSingleShot(true);

    connect(
        &max_velocity_hold_timer_,
        &QTimer::timeout,
        this,
        &TestExecutionController::maxVelocityHoldFinished);


    zero_velocity_hold_timer_.setSingleShot(true);

    connect(
        &zero_velocity_hold_timer_,
        &QTimer::timeout,
        this,
        &TestExecutionController::zeroVelocityHoldFinished);
}




void TestExecutionController::setTestCases(
    const std::vector<robot_motion_test_data::TestCase>& testCases)
{
    test_cases_ = testCases;
}




void TestExecutionController::setWaypoints(
    const QString& startWaypoint,
    const QString& goalWaypoint)
{
    start_waypoint_ = startWaypoint;
    goal_waypoint_ = goalWaypoint;
}



bool TestExecutionController::isRunning() const
{
    return running_;
}



void TestExecutionController::start()
{
    if (running_)
    {
        return;
    }


    if (test_cases_.empty())
    {
        return;
    }


    if (manual_waypoint_mode_)
    {
        if (manual_start_node_id_ < 0 ||
            manual_goal_node_id_ < 0)
        {
            emit statusChanged(
                "Enter valid manual start and goal vertex IDs.");

            return;
        }


        if (manual_start_node_id_ ==
            manual_goal_node_id_)
        {
            emit statusChanged(
                "Manual start and goal vertex IDs "
                "must be different.");

            return;
        }
    }
    else
    {
        if (start_waypoint_.isEmpty() ||
            goal_waypoint_.isEmpty())
        {
            emit statusChanged(
                "Select start and goal waypoints.");

            return;
        }


        if (start_waypoint_ ==
            goal_waypoint_)
        {
            emit statusChanged(
                "Start and goal waypoints must be different.");

            return;
        }
    }


    running_ = true;

    current_test_index_ = 0;

    current_iteration_ = 1;

    waiting_for_load_ = false;

    load_prompt_shown_ = false;

    phase_ = Phase::IDLE;

    braking_started_ = false;

    brake_released_ = false;

    return_motion_detected_ = false;

    maximum_velocity_hold_started_ = false;

    zero_velocity_hold_started_ = false;


    max_velocity_hold_timer_.stop();

    zero_velocity_hold_timer_.stop();


    emit statusChanged(
        "Test sequence started.");

    startCurrentTest();
}




void TestExecutionController::startCurrentTest()
{
    if (!running_)
    {
        return;
    }


    if (current_test_index_ < 0 ||
        current_test_index_ >=
            static_cast<int>(test_cases_.size()))
    {
        running_ = false;

        waiting_for_load_ = false;

        phase_ = Phase::IDLE;

        emit statusChanged(
            "All tests completed.");

        emit executionCompleted();

        return;
    }


    const auto& testCase =
        test_cases_[current_test_index_];


    if (testCase.load_condition ==
        robot_motion_test_data::LoadCondition::UNDER_LOAD)
    {
        if (!load_prompt_shown_)
        {
            waiting_for_load_ = true;

            load_prompt_shown_ = true;

            phase_ = Phase::IDLE;

            emit statusChanged(
                QString(
                    "Load required for %1. "
                    "Waiting for operator confirmation.")
                .arg(
                    QString::fromStdString(
                        testCase.name)));

            emit loadRequired(
                testCase.load_mass_kg);

            return;
        }


        if (waiting_for_load_)
        {
            return;
        }
    }


    waiting_for_load_ = false;

    clearCurrentData();


    phase_ = Phase::MOVING_TO_GOAL;

    braking_started_ = false;

    brake_released_ = false;

    return_motion_detected_ = false;

    maximum_velocity_hold_started_ = false;

    zero_velocity_hold_started_ = false;


    max_velocity_hold_timer_.stop();

    zero_velocity_hold_timer_.stop();


    emit testStarted(
        QString::fromStdString(
            testCase.name),
        current_iteration_);


    emit statusChanged(
        QString(
            "Running %1 iteration %2")
        .arg(
            QString::fromStdString(
                testCase.name))
        .arg(
            current_iteration_));


    emit statusChanged(
        QString(
            "Configuring motion: max velocity %1 m/s")
        .arg(
            testCase.max_velocity_mps,
            0,
            'f',
            3));


    if (!ros_client_->configureMotion(
            testCase.max_velocity_mps,
            testCase.acceleration_mps2))
    {
        running_ = false;

        waiting_for_load_ = false;

        phase_ = Phase::IDLE;

        emit statusChanged(
            "Failed to configure robot motion.");

        emit executionStopped();

        return;
    }


    emit statusChanged(
        "Motion configuration applied. "
        "Preparing test movement.");

    QThread::msleep(300);




    bool goalSent = false;


    if (manual_waypoint_mode_)
    {
        if (manual_goal_node_id_ < 0)
        {
            running_ = false;

            waiting_for_load_ = false;

            phase_ = Phase::IDLE;

            emit statusChanged(
                "Invalid manual goal vertex ID.");

            emit executionStopped();

            return;
        }


        ROS_INFO(
            "Using manual goal vertex ID: %d",
            manual_goal_node_id_);

        goalSent =
            ros_client_->sendGoalByNodeId(
                manual_goal_node_id_);
    }
    else
    {
        goalSent =
            ros_client_->sendGoal(
                start_waypoint_,
                goal_waypoint_);
    }


    if (!goalSent)
    {
        running_ = false;

        waiting_for_load_ = false;

        phase_ = Phase::IDLE;

        emit statusChanged(
            "Failed to send test goal.");

        emit executionStopped();

        return;
    }


   

    if (manual_waypoint_mode_)
    {
        emit statusChanged(
            QString(
                "Test started: vertex %1 -> vertex %2")
            .arg(manual_start_node_id_)
            .arg(manual_goal_node_id_));
    }
    else
    {
        emit statusChanged(
            QString(
                "Test started: %1 -> %2")
            .arg(start_waypoint_)
            .arg(goal_waypoint_));
    }
}



void TestExecutionController::loadAccepted()
{
    if (!running_)
    {
        return;
    }


    if (!waiting_for_load_)
    {
        return;
    }


    waiting_for_load_ = false;

    emit statusChanged(
        "Load confirmed. Starting test.");


    startCurrentTest();
}




void TestExecutionController::telemetryUpdated(
    double velocity,
    double distance,
    double time)
{
    if (!running_)
    {
        return;
    }


    if (current_test_index_ < 0 ||
        current_test_index_ >=
            static_cast<int>(test_cases_.size()))
    {
        return;
    }


    if (waiting_for_load_)
    {
        return;
    }


    const auto& testCase =
        test_cases_[current_test_index_];


    latest_distance_ = distance;

    latest_time_ = time;


    max_velocity_achieved_ =
        std::max(
            max_velocity_achieved_,
            velocity);


  

    if (previous_time_ > 0.0 &&
        time > previous_time_)
    {
        const double dt =
            time - previous_time_;

        if (dt > 0.0)
        {
            const double acceleration =
                (velocity -
                 previous_velocity_) /
                dt;


            acceleration_time_data_.append(
                QPointF(
                    time,
                    acceleration));


            emit accelerationDataChanged(
                acceleration_time_data_);
        }
    }


    previous_velocity_ = velocity;

    previous_time_ = time;



    if (phase_ == Phase::MOVING_TO_GOAL ||
        phase_ == Phase::HOLDING_MAX_VELOCITY ||
        phase_ == Phase::BRAKING ||
        phase_ == Phase::HOLDING_ZERO_VELOCITY)
    {
        velocity_time_data_.append(
            QPointF(
                time,
                velocity));

        velocity_distance_data_.append(
            QPointF(
                distance,
                velocity));

        emit velocityDataChanged(
            velocity_time_data_);

        emit velocityDistanceDataChanged(
            velocity_distance_data_);
    }




    if (braking_started_ &&
        !brake_released_ &&
        (testCase.braking_type ==
             robot_motion_test_data::BrakingType::STO ||
         testCase.braking_type ==
             robot_motion_test_data::BrakingType::SAFE_STOP ||
         testCase.braking_type ==
             robot_motion_test_data::BrakingType::NORMAL_BRAKING))
    {
        braking_distance_ =
            std::max(
                0.0,
                distance - braking_start_distance_);


        if (std::abs(velocity) <= 0.03)
        {
            braking_time_ =
                std::max(
                    0.0,
                    time - braking_start_time_);


         

            if (testCase.braking_type ==
                robot_motion_test_data::BrakingType::NORMAL_BRAKING)
            {
                brake_released_ = true;

                emit statusChanged(
                    "Robot reached zero velocity. "
                    "Normal braking completed. "
                    "Returning to start waypoint.");

                startReturnToStart();

                return;
            }


          

            releaseBrakeAndReturn();

            return;
        }
    }


   

    if (phase_ == Phase::MOVING_TO_GOAL &&
        velocity >= testCase.max_velocity_mps)
    {
     

        if (testCase.braking_type ==
            robot_motion_test_data::BrakingType::NONE)
        {
            

            if (!braking_started_)
            {
                braking_started_ = true;

                emit statusChanged(
                    QString(
                        "Maximum velocity reached: %1 m/s. "
                        "Continuing to goal waypoint: %2")
                    .arg(
                        velocity,
                        0,
                        'f',
                        3)
                    .arg(
                        goal_waypoint_));
            }

            return;
        }


   

        if (testCase.braking_type ==
            robot_motion_test_data::BrakingType::NORMAL_BRAKING)
        {
            if (!maximum_velocity_hold_started_)
            {
                maximum_velocity_hold_started_ = true;

                phase_ =
                    Phase::HOLDING_MAX_VELOCITY;

                emit statusChanged(
                    QString(
                        "Maximum velocity reached: %1 m/s. "
                        "Holding maximum velocity for 2 seconds "
                        "before normal braking.")
                    .arg(
                        velocity,
                        0,
                        'f',
                        3));

                startMaximumVelocityHold();
            }

            return;
        }


       

        if (testCase.braking_type ==
                robot_motion_test_data::BrakingType::STO ||
            testCase.braking_type ==
                robot_motion_test_data::BrakingType::SAFE_STOP)
        {
            if (!maximum_velocity_hold_started_)
            {
                maximum_velocity_hold_started_ = true;

                phase_ =
                    Phase::HOLDING_MAX_VELOCITY;

                emit statusChanged(
                    QString(
                        "Maximum velocity reached: %1 m/s. "
                        "Holding maximum velocity for 1 second.")
                    .arg(
                        velocity,
                        0,
                        'f',
                        3));

                startMaximumVelocityHold();
            }

            return;
        }
    }



    if (phase_ == Phase::BRAKING)
    {
        braking_distance_ =
            std::max(
                0.0,
                distance - braking_start_distance_);


        if (std::abs(velocity) <= 0.03)
        {
            braking_time_ =
                std::max(
                    0.0,
                    time - braking_start_time_);


            if (testCase.braking_type ==
                robot_motion_test_data::BrakingType::NORMAL_BRAKING)
            {
                brake_released_ = true;

                emit statusChanged(
                    "Robot reached zero velocity. "
                    "Normal braking completed. "
                    "Returning to start waypoint.");

                startReturnToStart();

                return;
            }


            if (!zero_velocity_hold_started_)
            {
                zero_velocity_hold_started_ = true;

                phase_ =
                    Phase::HOLDING_ZERO_VELOCITY;

                emit statusChanged(
                    "Robot reached zero velocity. "
                    "Holding stopped state for 1 second.");

                startZeroVelocityHold();
            }
        }

        return;
    }


 

    if (phase_ == Phase::HOLDING_ZERO_VELOCITY)
    {
        return;
    }



    if (phase_ == Phase::RETURNING_TO_START)
    {
        if (std::abs(velocity) > 0.05)
        {
            if (!return_motion_detected_)
            {
                return_motion_detected_ = true;

                emit statusChanged(
                    "Return motion detected.");
            }
        }

        return;
    }
}




void TestExecutionController::startMaximumVelocityHold()
{
    if (!running_)
    {
        return;
    }


    if (phase_ != Phase::HOLDING_MAX_VELOCITY)
    {
        return;
    }


    if (current_test_index_ < 0 ||
        current_test_index_ >=
            static_cast<int>(test_cases_.size()))
    {
        return;
    }


    const auto& testCase =
        test_cases_[current_test_index_];


  

    if (testCase.braking_type ==
        robot_motion_test_data::BrakingType::NORMAL_BRAKING)
    {
        max_velocity_hold_timer_.start(2000);
    }
    else
    {
        max_velocity_hold_timer_.start(1000);
    }
}



void TestExecutionController::stoFeedbackUpdated(
    bool active)
{
    sto_feedback_active_ = active;
}




void TestExecutionController::safeStopFeedbackUpdated(
    bool active)
{
    safe_stop_feedback_active_ = active;
}




void TestExecutionController::maxVelocityHoldFinished()
{
    if (!running_)
    {
        return;
    }


    if (current_test_index_ < 0 ||
        current_test_index_ >=
            static_cast<int>(test_cases_.size()))
    {
        return;
    }


    if (phase_ != Phase::HOLDING_MAX_VELOCITY)
    {
        return;
    }


    const auto& testCase =
        test_cases_[current_test_index_];


    if (testCase.braking_type !=
            robot_motion_test_data::BrakingType::STO &&
        testCase.braking_type !=
            robot_motion_test_data::BrakingType::SAFE_STOP &&
        testCase.braking_type !=
            robot_motion_test_data::BrakingType::NORMAL_BRAKING)
    {
        return;
    }


    activateBrakeAndStop();
}




void TestExecutionController::activateBrakeAndStop()
{
    if (!running_)
    {
        return;
    }


    if (current_test_index_ < 0 ||
        current_test_index_ >=
            static_cast<int>(test_cases_.size()))
    {
        return;
    }


    const auto& testCase =
        test_cases_[current_test_index_];


    /*
     * Start measuring braking distance/time at the exact
     * moment the braking action is activated.
     */

    braking_start_distance_ =
        latest_distance_;

    braking_start_time_ =
        latest_time_;

    braking_distance_ =
        0.0;

    braking_time_ =
        0.0;

    braking_started_ =
        true;

    brake_released_ =
        false;



    if (testCase.braking_type ==
        robot_motion_test_data::BrakingType::NORMAL_BRAKING)
    {
        emit statusChanged(
            "2 second maximum-velocity hold completed. "
            "Activating normal braking.");



        ros_client_->stopRobot();

        ros_client_->publishNormalBrakeStop();


        emit brakeActivated(
            "NORMAL BRAKING ACTIVATED",
            latest_time_,
            latest_distance_);


        phase_ =
            Phase::BRAKING;

        return;
    }


   

    if (testCase.braking_type ==
        robot_motion_test_data::BrakingType::STO)
    {
        emit statusChanged(
            "1 second maximum-velocity hold completed. "
            "Activating STO.");

        ros_client_->triggerSto();


        emit brakeActivated(
            "STO ACTIVATED",
            latest_time_,
            latest_distance_);
    }




    else if (
        testCase.braking_type ==
        robot_motion_test_data::BrakingType::SAFE_STOP)
    {
        emit statusChanged(
            "1 second maximum-velocity hold completed. "
            "Activating Safe-Stop.");

        ros_client_->triggerSafeStop();


        emit brakeActivated(
            "SAFE-STOP ACTIVATED",
            latest_time_,
            latest_distance_);
    }


  

    phase_ =
        Phase::MOVING_TO_GOAL;


    emit statusChanged(
        "Brake activation published. "
        "Continuing to goal waypoint.");
}




void TestExecutionController::releaseBrakeAndReturn()
{
    if (!running_)
    {
        return;
    }


    if (brake_released_)
    {
        return;
    }


    if (current_test_index_ < 0 ||
        current_test_index_ >=
            static_cast<int>(test_cases_.size()))
    {
        return;
    }


    const auto& testCase =
        test_cases_[current_test_index_];


    brake_released_ =
        true;


    if (testCase.braking_type ==
        robot_motion_test_data::BrakingType::STO)
    {
        ros_client_->releaseSto();

        emit statusChanged(
            "Robot reached zero velocity. "
            "STO released. "
            "Returning to start waypoint.");
    }
    else if (
        testCase.braking_type ==
        robot_motion_test_data::BrakingType::SAFE_STOP)
    {
        ros_client_->releaseSafeStop();

        emit statusChanged(
            "Robot reached zero velocity. "
            "Safe-Stop released. "
            "Returning to start waypoint.");
    }
    else
    {
        return;
    }


    startReturnToStart();
}



void TestExecutionController::startZeroVelocityHold()
{
    if (!running_)
    {
        return;
    }


    if (phase_ != Phase::HOLDING_ZERO_VELOCITY)
    {
        return;
    }


    zero_velocity_hold_timer_.start(1000);
}




void TestExecutionController::zeroVelocityHoldFinished()
{
    if (!running_)
    {
        return;
    }


    if (phase_ != Phase::HOLDING_ZERO_VELOCITY)
    {
        return;
    }


    emit statusChanged(
        "One-second stopped hold completed. "
        "Returning to start waypoint.");


    startReturnToStart();
}




void TestExecutionController::startReturnToStart()
{
    if (!running_)
    {
        return;
    }


    phase_ =
        Phase::RETURNING_TO_START;


    return_motion_detected_ =
        false;


    previous_velocity_ =
        0.0;

    previous_time_ =
        0.0;


    

    bool returnGoalSent =
        false;


    if (manual_waypoint_mode_)
    {
        if (manual_start_node_id_ < 0)
        {
            running_ = false;

            waiting_for_load_ = false;

            phase_ =
                Phase::IDLE;

            emit statusChanged(
                "Invalid manual start vertex ID.");

            emit executionStopped();

            return;
        }


        ROS_INFO(
            "Using manual start vertex ID for return: %d",
            manual_start_node_id_);


        returnGoalSent =
            ros_client_->sendGoalByNodeId(
                manual_start_node_id_);
    }
    else
    {
        returnGoalSent =
            ros_client_->sendGoal(
                "",
                start_waypoint_);
    }


    if (!returnGoalSent)
    {
        running_ = false;

        waiting_for_load_ = false;

        phase_ =
            Phase::IDLE;

        emit statusChanged(
            "Failed to send return-to-start goal.");

        emit executionStopped();

        return;
    }


    if (manual_waypoint_mode_)
    {
        emit statusChanged(
            QString(
                "Returning to start vertex: %1")
            .arg(manual_start_node_id_));
    }
    else
    {
        emit statusChanged(
            QString(
                "Returning to start waypoint: %1")
            .arg(start_waypoint_));
    }
}




void TestExecutionController::goalReached()
{
    if (!running_)
    {
        return;
    }


    if (current_test_index_ < 0 ||
        current_test_index_ >=
            static_cast<int>(test_cases_.size()))
    {
        return;
    }


    if (waiting_for_load_)
    {
        return;
    }


    const auto& testCase =
        test_cases_[current_test_index_];




    if (phase_ == Phase::MOVING_TO_GOAL)
    {
      

        if (braking_started_ &&
            !brake_released_ &&
            (testCase.braking_type ==
                 robot_motion_test_data::BrakingType::STO ||
             testCase.braking_type ==
                 robot_motion_test_data::BrakingType::SAFE_STOP))
        {
            emit statusChanged(
                "Goal reached while brake is active. "
                "Waiting for zero velocity before releasing brake.");

            return;
        }


        emit statusChanged(
            QString(
                "Goal waypoint reached: %1")
            .arg(goal_waypoint_));


        startReturnToStart();


        return;
    }




    if (phase_ == Phase::HOLDING_MAX_VELOCITY ||
        phase_ == Phase::BRAKING ||
        phase_ == Phase::HOLDING_ZERO_VELOCITY)
    {
        emit statusChanged(
            "Ignoring goal callback during brake sequence.");

        return;
    }



    if (phase_ == Phase::RETURNING_TO_START)
    {

        phase_ =
            Phase::IDLE;


        emit statusChanged(
            QString(
                "Return goal reached: %1")
            .arg(start_waypoint_));


        

        finishCurrentIteration();


        return;
    }
}


void TestExecutionController::setManualWaypoints(
    int startNodeId,
    int goalNodeId)
{
    manual_start_node_id_ =
        startNodeId;

    manual_goal_node_id_ =
        goalNodeId;

    manual_waypoint_mode_ =
        true;


    ROS_INFO(
        "Manual waypoint mode enabled: "
        "start=%d goal=%d",
        manual_start_node_id_,
        manual_goal_node_id_);
}




void TestExecutionController::clearManualWaypoints()
{
    manual_start_node_id_ =
        -1;

    manual_goal_node_id_ =
        -1;

    manual_waypoint_mode_ =
        false;


    ROS_INFO(
        "Manual waypoint mode disabled.");
}




void TestExecutionController::finishCurrentIteration()
{
    if (!running_)
    {
        return;
    }


    if (current_test_index_ < 0 ||
        current_test_index_ >=
            static_cast<int>(test_cases_.size()))
    {
        return;
    }


    const auto& testCase =
        test_cases_[current_test_index_];


    emit testFinished(
        QString::fromStdString(
            testCase.name));


  

    if (current_iteration_ >= testCase.iterations)
    {
        robot_motion_test_data::DataAccessLayer data_access;


        if (!data_access.initialize())
        {
            emit statusChanged(
                "MongoDB connection failed. "
                "Test result was not saved.");
        }
        else
        {
            robot_motion_test_data::TestResult result;


            result.test_case_name =
                testCase.name;

            if (manual_waypoint_mode_)
        {
         result.start_waypoint_id =
           std::to_string(manual_start_node_id_);

          result.goal_waypoint_id =
            std::to_string(manual_goal_node_id_);
        }
       else
         {
         result.start_waypoint_id =
            start_waypoint_.toStdString();

          result.goal_waypoint_id =
             goal_waypoint_.toStdString();
            }



            


            result.iterations_run =
                current_iteration_;


            result.max_velocity_achieved_mps =
                max_velocity_achieved_;


            result.min_velocity_achieved_mps =
                0.0;


            if (!velocity_time_data_.isEmpty())
            {
                result.min_velocity_achieved_mps =
                    velocity_time_data_.first().y();


                for (const auto& sample :
                     velocity_time_data_)
                {
                    result.min_velocity_achieved_mps =
                        std::min(
                            result.min_velocity_achieved_mps,
                            sample.y());
                }
            }


          

            double highest_valid_acceleration =
                0.0;


            for (const auto& sample :
                 acceleration_time_data_)
            {
                const double acceleration =
                    sample.y();


                if (acceleration >= 0.0 &&
                    acceleration <=
                        testCase.acceleration_mps2)
                {
                    highest_valid_acceleration =
                        std::max(
                            highest_valid_acceleration,
                            acceleration);
                }
            }


            result.acceleration_achieved_mps2 =
                highest_valid_acceleration;


            result.load_condition =
                testCase.load_condition;


            result.load_mass_kg =
                testCase.load_mass_kg;


            result.braking_type =
                testCase.braking_type;


            result.braking_distance_m =
                braking_distance_;


            result.braking_time_s =
                braking_time_;


            result.status =
                robot_motion_test_data::TestStatus::COMPLETED;



            result.velocity_time.reserve(
                velocity_time_data_.size());


            for (const auto& sample :
                 velocity_time_data_)
            {
                result.velocity_time.emplace_back(
                    sample.x(),
                    sample.y());
            }


            result.velocity_distance.reserve(
                velocity_distance_data_.size());


            for (const auto& sample :
                 velocity_distance_data_)
            {
                result.velocity_distance.emplace_back(
                    sample.x(),
                    sample.y());
            }


            result.acceleration_time.reserve(
                acceleration_time_data_.size());


            for (const auto& sample :
                 acceleration_time_data_)
            {
                result.acceleration_time.emplace_back(
                    sample.x(),
                    sample.y());
            }


            if (!data_access.saveTestResult(result))
            {
                emit statusChanged(
                    "Failed to save test result to MongoDB.");
            }
            else
            {
                emit statusChanged(
                    "Test result saved to MongoDB.");
            }
        }
    }



    if (current_iteration_ <
        testCase.iterations)
    {
        ++current_iteration_;


        emit statusChanged(
            QString(
                "Starting iteration %1")
            .arg(
                current_iteration_));


       

        startCurrentTest();

        return;
    }


   

    ++current_test_index_;


    if (current_test_index_ >=
        static_cast<int>(test_cases_.size()))
    {
        running_ = false;

        waiting_for_load_ = false;

        load_prompt_shown_ = false;

        phase_ =
            Phase::IDLE;

        emit statusChanged(
            "All tests completed.");

        emit executionCompleted();

        return;
    }


    current_iteration_ =
        1;

    load_prompt_shown_ =
        false;

    waiting_for_load_ =
        false;

    phase_ =
        Phase::IDLE;

    return_motion_detected_ =
        false;


    emit statusChanged(
        "Previous test completed. "
        "Preparing next test case.");


    startCurrentTest();
}




void TestExecutionController::clearCurrentData()
{
    max_velocity_hold_timer_.stop();

    zero_velocity_hold_timer_.stop();


    velocity_time_data_.clear();

    velocity_distance_data_.clear();

    acceleration_time_data_.clear();


    braking_started_ =
        false;

    brake_released_ =
        false;

    return_motion_detected_ =
        false;

    sto_feedback_active_ =
        false;

    safe_stop_feedback_active_ =
        false;


    braking_start_distance_ =
        0.0;

    braking_distance_ =
        0.0;

    braking_start_time_ =
        0.0;

    braking_time_ =
        0.0;

    max_velocity_achieved_ =
        0.0;


    previous_velocity_ =
        0.0;

    previous_time_ =
        0.0;

    latest_distance_ =
        0.0;

    latest_time_ =
        0.0;


    maximum_velocity_hold_started_ =
        false;

    zero_velocity_hold_started_ =
        false;


    emit velocityDataChanged(
        velocity_time_data_);

    emit velocityDistanceDataChanged(
        velocity_distance_data_);

    emit accelerationDataChanged(
        acceleration_time_data_);
}




void TestExecutionController::stop()
{
    if (!running_)
    {
        return;
    }


    max_velocity_hold_timer_.stop();

    zero_velocity_hold_timer_.stop();


    ros_client_->stopRobot();


    running_ =
        false;

    waiting_for_load_ =
        false;

    load_prompt_shown_ =
        false;

    phase_ =
        Phase::IDLE;

    braking_started_ =
        false;

    brake_released_ =
        false;

    return_motion_detected_ =
        false;


    clearCurrentData();


    emit executionStopped();


    emit statusChanged(
        "Testing stopped.");
}