#include "robot_motion_test_data/data_access_layer.h"

#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <bsoncxx/types.hpp>

#include <iostream>
#include <stdexcept>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <mongocxx/options/index.hpp>
#include <mongocxx/instance.hpp>

namespace
{
    mongocxx::instance& mongoInstance()
    {
        static mongocxx::instance instance;
        return instance;
    }
}
namespace robot_motion_test_data
{

    using bsoncxx::builder::basic::array;
    using bsoncxx::builder::basic::document;
    using bsoncxx::builder::basic::kvp;
    using bsoncxx::builder::basic::make_document;

    template <typename T>
    bool hasResult(const T& result)
   {
    return static_cast<bool>(result);
   }

    static std::string brakingTypeToString(BrakingType type)
    {
        switch(type)
        {
            case BrakingType::STO:
                 return "STO";

            case BrakingType::SAFE_STOP:
                 return "SAFE_STOP";

            case BrakingType::NONE:
                return "NONE";

        }

        return "NONE";
    }

    static BrakingType stringToBrakingType(const std::string& value)
    {
          if (value == "STO")
        return BrakingType::STO;

    if (value == "SAFE_STOP")
        return BrakingType::SAFE_STOP;

    return BrakingType::NONE;
    }

    static std::string loadConditionToString(LoadCondition condition)
    {
        switch(condition)
        {
            case LoadCondition::NO_LOAD:
            return "NO_LOAD";

            case LoadCondition::UNDER_LOAD:
            return "UNDER_LOAD";
        }

        return "NO_LOAD";
    }

    static LoadCondition stringToLoadCondition(const std::string& value)
    {

    if (value == "UNDER_LOAD")
        return LoadCondition::UNDER_LOAD;

    return LoadCondition::NO_LOAD;
    
    }

    static std::string statusToString(TestStatus status)
    {
        switch(status)
        {
            case TestStatus::COMPLETED:
                 return "COMPLETED";
            case TestStatus::STOPPED:
                 return "STOPPED";
        }

        return "COMPLETED";
    }

    static TestStatus stringToTestStatus(
        const std::string& value)
    {
        if (value == "STOPPED")
            return TestStatus::STOPPED;

        return TestStatus::COMPLETED;
    }


DataAccessLayer::DataAccessLayer( const std::string& uri, const std::string& database_name)
                : uri_(uri),
                  database_name_(database_name),
                  client_(nullptr),
                  initialized_(false)
        {
        }

bool DataAccessLayer::initialize()
{
    try 
    {   
        mongoInstance();

        client_ = std::make_unique<mongocxx::client>(
            mongocxx::uri{uri_});

        database_ = (*client_)[database_name_];

        auto test_cases = database_["test_cases"];

        auto index_spec = make_document(kvp("name", 1));

  

        test_cases.create_index(index_spec.view(),
            mongocxx::options::index{}
                .unique(true));

        initialized_ = true;

        std::cout
            << "[MongoDB] Connected to database:"
            << database_name_
            << std::endl;
        
        return true;
    }
    catch (const std::exception& e)
    {

        std::cerr
            << "[MongoDB] Initialization failed"
            << e.what()
            << std::endl;

        initialized_ = false;
        return false;
    }
}

DataAccessLayer::~DataAccessLayer()
{
    client_.reset();
}

bool DataAccessLayer::isConnected() const
{
    return initialized_;
}

bool DataAccessLayer::createTestCase(
    const TestCase& test_case)
{
    if (!initialized_)
    {
        std::cerr
            << "[MongoDB] Not initialized."
            << std::endl;

        return false;
    }

    try
    {
        auto collection = database_["test_cases"];

        auto document_builder = document{};

        document_builder.append(
            kvp("name", test_case.name));

        document_builder.append(
            kvp("max_velocity_mps",
                test_case.max_velocity_mps));

        document_builder.append(
            kvp("acceleration_mps2",
                test_case.acceleration_mps2));

        document_builder.append(
            kvp("braking_type",
                brakingTypeToString(
                    test_case.braking_type)));

        document_builder.append(
            kvp("load_condition",
                loadConditionToString(
                    test_case.load_condition)));

        
        if (test_case.load_condition ==
            LoadCondition::UNDER_LOAD)
        {
            document_builder.append(
                kvp("load_mass_kg",
                    test_case.load_mass_kg));
        
        }
        else
        {

            document_builder.append(
                kvp("load_mass_kg", bsoncxx::types::b_null{}));
            
        }

        document_builder.append(
            kvp("iterations", test_case.iterations));

        auto now = bsoncxx::types::b_date{
                     std::chrono::system_clock::now()

        };

        document_builder.append(
            kvp("created_at", now));
        document_builder.append(
            kvp("updated_at", now));

        auto result = collection.insert_one(document_builder.view());

        return hasResult(result);
    }
    
    catch(const std::exception& e)
    {
       std::cerr
           <<"[MongoDB] Create test case failed:"
           << e.what()
           << std::endl;

           return false;
    }
}


bool DataAccessLayer::updateTestCase(
    const std::string& name,
    const TestCase& test_case)
{
    if (!initialized_)
    {
        return false;
    }

    try
    {
        auto collection = database_["test_cases"];

        auto filter =
            make_document(
                kvp("name", name));

        auto update_document = document{};

        update_document.append(
            kvp("name", test_case.name));

        update_document.append(
            kvp("max_velocity_mps",
                test_case.max_velocity_mps));

        update_document.append(
            kvp("acceleration_mps2",
                test_case.acceleration_mps2));

        update_document.append(
            kvp("braking_type",
                brakingTypeToString(
                    test_case.braking_type)));

        update_document.append(
            kvp("load_condition",
                loadConditionToString(
                    test_case.load_condition)));

        if (test_case.load_condition ==
            LoadCondition::UNDER_LOAD)
        {
            update_document.append(
                kvp("load_mass_kg",
                    test_case.load_mass_kg));
        }
        else
        {
            update_document.append(
                kvp("load_mass_kg",
                    bsoncxx::types::b_null{}));
        }

        update_document.append(
            kvp("iterations",
                test_case.iterations));

        update_document.append(
            kvp("updated_at",
                bsoncxx::types::b_date{
                    std::chrono::system_clock::now()
                }));

        auto update =
            make_document(
                kvp("$set",
                    update_document.view()));

        auto result =
            collection.update_one(
                filter.view(),
                update.view());

        return hasResult(result) &&
               result->matched_count() > 0;
        }

        catch (const std::exception& e)
        {
            std::cerr
             <<"[MongoDB] Update test case failed:"
             << e.what()
             << std::endl;

          return false;
        }
    }

    bool DataAccessLayer::deleteTestCase(
    const std::string& name)
{
    if (!initialized_)
    {
        return false;
    }

    try
    {
        auto collection = database_["test_cases"];

        auto filter =
            make_document(
                kvp("name", name));

        auto result =
            collection.delete_one(
                filter.view());

        return hasResult(result) &&
               result->deleted_count() > 0;
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[MongoDB] Delete test case failed: "
            << e.what()
            << std::endl;

        return false;
    }
}

bool DataAccessLayer::getTestCase(
    const std::string& name,
    TestCase& test_case)
{
    if (!initialized_)
    {
        return false;
    }

    try
    {
        auto collection = database_["test_cases"];

        auto filter =
            make_document(
                kvp("name", name));

        auto result =
            collection.find_one(
                filter.view());

        if (!result)
        {
            return false;
        }

        auto view = result->view();

        test_case.name =
            view["name"].get_string().value.to_string();

        test_case.max_velocity_mps =
            view["max_velocity_mps"].get_double().value;

        test_case.acceleration_mps2 =
            view["acceleration_mps2"].get_double().value;

        test_case.braking_type =
            stringToBrakingType(
                view["braking_type"]
                    .get_string()
                    .value
                    .to_string());

        test_case.load_condition =
            stringToLoadCondition(
                view["load_condition"]
                    .get_string()
                    .value
                    .to_string());


        auto load_mass =
                view["load_mass_kg"];


        if (load_mass &&
    load_mass.type() != bsoncxx::type::k_null)
            {
                test_case.load_mass_kg = 
                     load_mass.get_double().value;

            }
        else
        {
            test_case.load_mass_kg = 0.0;
        }

            test_case.iterations =
                     view["iterations"]
                    .get_int32()
                    .value;

          return true;
        
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[MongoDB] get  test case failed:"
            << e.what()
            << std::endl;
    

      return false;
    }
}
std::vector<TestCase>
DataAccessLayer::getAllTestCases()
{
    std::vector<TestCase> test_cases;

    if (!initialized_)
    {
        return test_cases;
    }

    try
    {
        auto collection = database_["test_cases"];

        auto cursor = collection.find({});

        for (auto&& view : cursor)
        {
            TestCase test_case;

            test_case.name =
                view["name"]
                    .get_string()
                    .value
                    .to_string();

            test_case.max_velocity_mps =
                view["max_velocity_mps"]
                    .get_double()
                    .value;

            test_case.acceleration_mps2 =
                view["acceleration_mps2"]
                    .get_double()
                    .value;

            test_case.braking_type =
                stringToBrakingType(
                    view["braking_type"]
                        .get_string()
                        .value
                        .to_string());

            test_case.load_condition =
                stringToLoadCondition(
                    view["load_condition"]
                        .get_string()
                        .value
                        .to_string());

            auto load_mass = view["load_mass_kg"];

            if (load_mass &&
                load_mass.type() != bsoncxx::type::k_null)
            {
                test_case.load_mass_kg =
                    load_mass.get_double()
                        .value;
            }
            else
            {
                test_case.load_mass_kg = 0.0;
            }

            test_case.iterations =
                view["iterations"]
                    .get_int32()
                    .value;

            test_cases.push_back(test_case);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[MongoDB] Get all test cases failed: "
            << e.what()
            << std::endl;
    }

    return test_cases;
}

bool DataAccessLayer::saveTestResult(
    const TestResult& result)
{
    if (!initialized_)
    {
        return false;
    }

    try
    {
        auto collection =
            database_["test_results"];

        auto document_builder = document{};

        document_builder.append(
            kvp("test_case_name",
                result.test_case_name));

        document_builder.append(
            kvp("executed_at",
                bsoncxx::types::b_date{
                    std::chrono::system_clock::now()
                }));

        document_builder.append(
            kvp("start_waypoint_id",
                result.start_waypoint_id));

        document_builder.append(
            kvp("goal_waypoint_id",
                result.goal_waypoint_id));

        document_builder.append(
            kvp("iterations_run",
                result.iterations_run));

        document_builder.append(
            kvp("max_velocity_achieved_mps",
                result.max_velocity_achieved_mps));

        document_builder.append(
            kvp("min_velocity_achieved_mps",
                result.min_velocity_achieved_mps));

        document_builder.append(
            kvp("acceleration_achieved_mps2",
                result.acceleration_achieved_mps2));

        document_builder.append(
            kvp("load_condition",
                loadConditionToString(
                    result.load_condition)));

        if (result.load_condition ==
            LoadCondition::UNDER_LOAD)
        {
            document_builder.append(
                kvp("load_mass_kg",
                    result.load_mass_kg));
        }
        else
        {
            document_builder.append(
                kvp("load_mass_kg",
                    bsoncxx::types::b_null{}));
        }

        document_builder.append(
            kvp("braking_type",
                brakingTypeToString(
                    result.braking_type)));

        document_builder.append(
            kvp("braking_distance_m",
                result.braking_distance_m));

        document_builder.append(
            kvp("braking_time_s",
                result.braking_time_s));

        document_builder.append(
            kvp("status",
                statusToString(
                    result.status)));


        array velocity_time_array;

        for (const auto& sample :
             result.velocity_time)
        {
            velocity_time_array.append(
                make_document(
                    kvp("t", sample.first),
                    kvp("v", sample.second)));
        }

        document_builder.append(
            kvp("velocity_time_series",
                velocity_time_array.extract()));


         array velocity_distance_array;

        for (const auto& sample :
             result.velocity_distance)
        {
            velocity_distance_array.append(
                make_document(
                    kvp("d", sample.first),
                    kvp("v", sample.second)));
        }

        document_builder.append(
            kvp("velocity_distance_series",
                velocity_distance_array.extract()));
        
                
         array acceleration_time_array;

        for (const auto& sample :
             result.acceleration_time)
        {
            acceleration_time_array.append(
                make_document(
                    kvp("t", sample.first),
                    kvp("a", sample.second)));
        }

        document_builder.append(
            kvp("acceleration_time_series",
                acceleration_time_array.extract()));

        auto insert_result =
            collection.insert_one(
                document_builder.view());

        return hasResult(insert_result);
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[MongoDB] Save test result failed: "
            << e.what()
            << std::endl;

        return false;
    }
}

std::vector<TestResult>
DataAccessLayer::getAllTestResults()
{
    std::vector<TestResult> results;

    if (!initialized_)
    {
        return results;
    }

    try
    {
        auto collection =
            database_["test_results"];

        auto cursor =
            collection.find({});

        for (auto&& view : cursor)
        {
            TestResult result;

            if (view["test_case_name"])
            {
                result.test_case_name =
                    view["test_case_name"]
                        .get_string()
                        .value
                        .to_string();
            }

          if (view["executed_at"] &&
    view["executed_at"].type() ==
        bsoncxx::type::k_date)
{
    const auto date_value =
        view["executed_at"]
            .get_date()
            .value;

    const std::time_t time =
        static_cast<std::time_t>(
            date_value.count() / 1000);

    std::tm* utc_time =
        std::gmtime(&time);

    if (utc_time != nullptr)
    {
        std::ostringstream stream;

        stream
            << std::put_time(
                utc_time,
                "%Y-%m-%d %H:%M:%S UTC");

        result.executed_at =
            stream.str();
    }
}

            if (view["start_waypoint_id"])
            {
                result.start_waypoint_id =
                    view["start_waypoint_id"]
                        .get_string()
                        .value
                        .to_string();
            }

            if (view["goal_waypoint_id"])
            {
                result.goal_waypoint_id =
                    view["goal_waypoint_id"]
                        .get_string()
                        .value
                        .to_string();
            }

            if (view["iterations_run"])
            {
                result.iterations_run =
                    view["iterations_run"]
                        .get_int32()
                        .value;
            }

            if (view["max_velocity_achieved_mps"])
            {
                result.max_velocity_achieved_mps =
                    view["max_velocity_achieved_mps"]
                        .get_double()
                        .value;
            }

            if (view["min_velocity_achieved_mps"])
            {
                result.min_velocity_achieved_mps =
                    view["min_velocity_achieved_mps"]
                        .get_double()
                        .value;
            }

            if (view["acceleration_achieved_mps2"])
            {
                result.acceleration_achieved_mps2 =
                    view["acceleration_achieved_mps2"]
                        .get_double()
                        .value;
            }

            if (view["load_condition"])
            {
                result.load_condition =
                    stringToLoadCondition(
                        view["load_condition"]
                            .get_string()
                            .value
                            .to_string());
            }

            auto load_mass =
                view["load_mass_kg"];

            if (load_mass &&
                load_mass.type() !=
                    bsoncxx::type::k_null)
            {
                result.load_mass_kg =
                    load_mass
                        .get_double()
                        .value;
            }

            if (view["braking_type"])
            {
                result.braking_type =
                    stringToBrakingType(
                        view["braking_type"]
                            .get_string()
                            .value
                            .to_string());
            }

            if (view["braking_distance_m"])
            {
                result.braking_distance_m =
                    view["braking_distance_m"]
                        .get_double()
                        .value;
            }

            if (view["braking_time_s"])
            {
                result.braking_time_s =
                    view["braking_time_s"]
                        .get_double()
                        .value;
            }

            if (view["status"])
            {
                result.status =
                    stringToTestStatus(
                        view["status"]
                            .get_string()
                            .value
                            .to_string());
            }

            auto velocity_time =
                view["velocity_time_series"];

            if (velocity_time &&
                velocity_time.type() ==
                    bsoncxx::type::k_array)
            {
                for (auto&& item :
                     velocity_time
                         .get_array()
                         .value)
                {
                    auto sample =
                        item.get_document().view();

                    result.velocity_time.emplace_back(
                        sample["t"]
                            .get_double()
                            .value,
                        sample["v"]
                            .get_double()
                            .value);
                }
            }

            auto velocity_distance =
                view["velocity_distance_series"];

            if (velocity_distance &&
                velocity_distance.type() ==
                    bsoncxx::type::k_array)
            {
                for (auto&& item :
                     velocity_distance
                         .get_array()
                         .value)
                {
                    auto sample =
                        item.get_document().view();

                    result.velocity_distance.emplace_back(
                        sample["d"]
                            .get_double()
                            .value,
                        sample["v"]
                            .get_double()
                            .value);
                }
            }

            auto acceleration_time =
                view["acceleration_time_series"];

            if (acceleration_time &&
                acceleration_time.type() ==
                    bsoncxx::type::k_array)
            {
                for (auto&& item :
                     acceleration_time
                         .get_array()
                         .value)
                {
                    auto sample =
                        item.get_document().view();

                    result.acceleration_time.emplace_back(
                        sample["t"]
                            .get_double()
                            .value,
                        sample["a"]
                            .get_double()
                            .value);
                }
            }

            results.push_back(result);
        }
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[MongoDB] Get all test results failed: "
            << e.what()
            << std::endl;
    }

    return results;
}

}
    

    


