#ifndef ROBOT_MOTION_TEST_DATA_DATA_ACCESS_LAYER_H
#define ROBOT_MOTION_TEST_DATA_DATA_ACCESS_LAYER_H

#include "robot_motion_test_data/models.h"

#include <mongocxx/instance.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>

#include <memory>
#include <string>
#include <vector>

namespace robot_motion_test_data
{
    class DataAccessLayer
    {
    public:
        DataAccessLayer(
            const std::string& uri = "mongodb://127.0.0.1:27017",
            const std::string& database_name = "robot_ce_testing");
        ~DataAccessLayer();

        bool initialize();
        bool isConnected() const;
        bool createTestCase(const TestCase& test_case);
        bool updateTestCase(const std::string& name, const TestCase& test_case);
        bool deleteTestCase(const std::string& name);
        bool getTestCase(const std::string& name, TestCase& test_case);
        std::vector<TestCase> getAllTestCases();
        bool saveTestResult(const TestResult& result);
        std::vector<TestResult> getAllTestResults();
    private:
        std::string uri_;
        std::string database_name_;
        std::unique_ptr<mongocxx::client> client_;
        mongocxx::database database_;
        bool initialized_;
    };
}
#endif