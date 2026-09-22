#ifndef ROBOT_MOTION_TEST_DATA_MODELS_H
#define ROBOT_MOTION_TEST_DATA_MODELS_H

#include <string>
#include <utility>
#include <vector>

namespace robot_motion_test_data
{
    enum class BrakingType
    {
        STO,
        SAFE_STOP,
        NORMAL_BRAKING,
        NONE
    };

    enum class LoadCondition
    {
        NO_LOAD,
        UNDER_LOAD
    };

    enum class TestStatus
    {
        COMPLETED,
        STOPPED
    };

    struct TestCase
    {
        std::string name;
        double max_velocity_mps = 0.0;
        double acceleration_mps2 = 0.0;
        BrakingType braking_type = BrakingType::NONE;
        LoadCondition load_condition = LoadCondition::NO_LOAD;
        double load_mass_kg = 0.0;
        int iterations = 1;
    };

    struct TestResult
    {
        std::string test_case_name;
        std::string executed_at;

        std::string start_waypoint_id;
        std::string goal_waypoint_id;

        int iterations_run = 0;

        double max_velocity_achieved_mps = 0.0;
        double min_velocity_achieved_mps = 0.0;
        double acceleration_achieved_mps2 = 0.0;

        LoadCondition load_condition =
            LoadCondition::NO_LOAD;

        double load_mass_kg = 0.0;

        BrakingType braking_type =
            BrakingType::NONE;

        double braking_distance_m = 0.0;
        double braking_time_s = 0.0;

        TestStatus status =
            TestStatus::COMPLETED;

        std::vector<std::pair<double, double>>
            velocity_time;

        std::vector<std::pair<double, double>>
            velocity_distance;

        std::vector<std::pair<double, double>>
            acceleration_time;
    };
}

#endif