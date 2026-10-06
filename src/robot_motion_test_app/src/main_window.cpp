#include "robot_motion_test_app/main_window.h"

#include "robot_motion_test_app/dialogs.h"
#include "robot_motion_test_app/graph_widget.h"
#include "robot_motion_test_app/ros_client.h"
#include "robot_motion_test_app/test_execution_controller.h"

#include "ui_main_window.h"

#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <QDateTime>
#include <QLineEdit>
#include <QRadioButton>
#include <QDir>
#include <QFile>
#include <QImage>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <utility>

// Creates the main application window.
MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui_(new Ui::MainWindow),
      data_access_(
          new robot_motion_test_data::DataAccessLayer),
      ros_client_(new RosClient(this)),
      execution_controller_(
          new TestExecutionController(
              ros_client_.get(),
              this)),
      velocity_time_graph_(nullptr),
      velocity_distance_graph_(nullptr),
      acceleration_time_graph_(nullptr),
      live_start_time_(0.0),
      live_start_distance_(0.0),
      live_previous_time_(0.0),
      live_previous_velocity_(0.0),
      live_telemetry_initialized_(false),
      warning_widget_(nullptr),
      mixed_order_overridden_(false),
      testing_active_(false)
{
    ui_->setupUi(this);

    // ---------------------------------------------------------
    // Manual vertex mode controls
    // ---------------------------------------------------------
    graph_waypoint_mode_radio_ =
        new QRadioButton(
            "Graph Waypoints",
            ui_->leftSidebar);

    manual_waypoint_mode_radio_ =
        new QRadioButton(
            "Manual Vertex IDs",
            ui_->leftSidebar);

    graph_waypoint_mode_radio_->setChecked(true);

    manual_start_vertex_edit_ =
        new QLineEdit(
            ui_->leftSidebar);

    manual_goal_vertex_edit_ =
        new QLineEdit(
            ui_->leftSidebar);

    manual_start_vertex_edit_->setPlaceholderText(
        "Start vertex ID");

    manual_goal_vertex_edit_->setPlaceholderText(
        "Goal vertex ID");

    manual_start_vertex_edit_->setEnabled(false);
    manual_goal_vertex_edit_->setEnabled(false);

    QWidget* manual_waypoint_widget =
        new QWidget(ui_->leftSidebar);

    QVBoxLayout* manual_layout =
        new QVBoxLayout(
            manual_waypoint_widget);

    manual_layout->setContentsMargins(
        8,
        8,
        8,
        8);

    manual_layout->setSpacing(6);

    manual_layout->addWidget(
        graph_waypoint_mode_radio_);

    manual_layout->addWidget(
        manual_waypoint_mode_radio_);

    manual_layout->addWidget(
        manual_start_vertex_edit_);

    manual_layout->addWidget(
        manual_goal_vertex_edit_);

    if (QVBoxLayout* left_layout =
            qobject_cast<QVBoxLayout*>(
                ui_->leftSidebar->layout()))
    {
        left_layout->insertWidget(
            0,
            manual_waypoint_widget);
    }

    connect(
        ros_client_.get(),
        &RosClient::waypointsUpdated,
        this,
        &MainWindow::updateWaypointList);

    updateWaypointList();

    ui_->leftSidebar->setMinimumWidth(0);
    ui_->leftSidebar->setMaximumWidth(
        QWIDGETSIZE_MAX);

    ui_->rightSidebar->setMinimumWidth(0);
    ui_->rightSidebar->setMaximumWidth(
        QWIDGETSIZE_MAX);

    ui_->centerArea->setMinimumWidth(0);
    ui_->centerArea->setMaximumWidth(
        QWIDGETSIZE_MAX);

    ui_->bodyLayout->setStretch(
        0,
        2);

    ui_->bodyLayout->setStretch(
        1,
        6);

    ui_->bodyLayout->setStretch(
        2,
        2);

    velocity_time_graph_ =
        new GraphWidget(
            "Velocity vs Time",
            "Time (s)",
            "Velocity (m/s)",
            ui_->centerArea);

    velocity_distance_graph_ =
        new GraphWidget(
            "Velocity vs Distance",
            "Distance (m)",
            "Velocity (m/s)",
            ui_->centerArea);

    acceleration_time_graph_ =
        new GraphWidget(
            "Acceleration vs Time",
            "Time (s)",
            "Acceleration (m/s²)",
            ui_->centerArea);

    ui_->centerLayout->addWidget(
        velocity_time_graph_,
        1);

    ui_->centerLayout->addWidget(
        velocity_distance_graph_,
        1);

    ui_->centerLayout->addWidget(
        acceleration_time_graph_,
        1);

    /*
     * Direct telemetry -> graph connection.
     *
     * We intentionally do not use the controller's graph
     * signals here. The graphs consume the same telemetry
     * that is already proven to reach the Qt application.
     */
    connect(
        ros_client_.get(),
        &RosClient::velocityUpdated,
        this,
        &MainWindow::updateLiveTelemetry);

    ui_->startWaypointComboBox
        ->setEditable(true);

    ui_->goalWaypointComboBox
        ->setEditable(true);

    ui_->startWaypointComboBox
        ->setInsertPolicy(
            QComboBox::NoInsert);

    ui_->goalWaypointComboBox
        ->setInsertPolicy(
            QComboBox::NoInsert);

    connect(
        manual_waypoint_mode_radio_,
        &QRadioButton::toggled,
        this,
        [this](bool manual)
        {
            manual_start_vertex_edit_->setEnabled(
                manual);

            manual_goal_vertex_edit_->setEnabled(
                manual);

            ui_->startWaypointComboBox
                ->setEnabled(!manual);

            ui_->goalWaypointComboBox
                ->setEnabled(!manual);

            showMixedLoadWarningIfNeeded();
        });

    connect(
        ui_->startWaypointComboBox,
        &QComboBox::currentTextChanged,
        this,
        [this](const QString&)
        {
            showMixedLoadWarningIfNeeded();
        });

    connect(
        ui_->goalWaypointComboBox,
        &QComboBox::currentTextChanged,
        this,
        [this](const QString&)
        {
            showMixedLoadWarningIfNeeded();
        });

    connect(
        manual_start_vertex_edit_,
        &QLineEdit::textChanged,
        this,
        [this](const QString&)
        {
            showMixedLoadWarningIfNeeded();
        });

    connect(
        manual_goal_vertex_edit_,
        &QLineEdit::textChanged,
        this,
        [this](const QString&)
        {
            showMixedLoadWarningIfNeeded();
        });

    connect(
        ui_->addTestCaseButton,
        &QPushButton::clicked,
        this,
        &MainWindow::addTestCase);

    connect(
        ui_->startButton,
        &QPushButton::clicked,
        this,
        &MainWindow::startTesting);

    connect(
        ui_->stopButton,
        &QPushButton::clicked,
        this,
        &MainWindow::stopTesting);

    // Create the export button programmatically so the export feature
    // does not depend on a regenerated ui_main_window.h file.
    QPushButton* export_button =
        new QPushButton(
            "EXPORT",
            ui_->leftSidebar);

    export_button->setFixedHeight(43);

    if (QVBoxLayout* left_layout =
            qobject_cast<QVBoxLayout*>(
                ui_->leftSidebar->layout()))
    {
        left_layout->insertWidget(
            1,
            export_button);
    }
    else
    {
        export_button->hide();
    }

    connect(
        export_button,
        &QPushButton::clicked,
        this,
        &MainWindow::exportResults);

    connect(
        execution_controller_.get(),
        &TestExecutionController::testStarted,
        this,
        &MainWindow::testStarted);

    connect(
        execution_controller_.get(),
        &TestExecutionController::executionStopped,
        this,
        &MainWindow::executionStopped);

    connect(
        execution_controller_.get(),
        &TestExecutionController::executionCompleted,
        this,
        &MainWindow::executionCompleted);

    connect(
        execution_controller_.get(),
        &TestExecutionController::statusChanged,
        this,
        [this](const QString& status)
        {
            ui_->statusLabel->setText(
                "Status: " + status);
        });

    connect(
        execution_controller_.get(),
        &TestExecutionController::loadRequired,
        this,
        [this](double massKg)
        {
            LoadPromptDialog dialog(
                massKg,
                this);

            const int result = dialog.exec();

            if (result == QDialog::Accepted)
            {
                execution_controller_->loadAccepted();
            }
            else
            {
                execution_controller_->stop();
            }
        });

    connect(
        execution_controller_.get(),
        &TestExecutionController::brakeActivated,
        this,
        &MainWindow::brakeActivated);

    ui_->stopButton->setEnabled(false);

    ui_->statusLabel->setText(
        "Status: IDLE");

    ui_->currentTestLabel->setText(
        "Current Test: --");

    ui_->iterationLabel->setText(
        "Iteration: --");

    ui_->velocityLabel->setText(
        "Velocity: --");

    ui_->distanceLabel->setText(
        "Distance: --");

    if (!data_access_->initialize())
    {
        QMessageBox::warning(
            this,
            "MongoDB",
            "Could not connect to MongoDB.");
    }

    updateWaypointList();

    loadTestCases();

    updateQueueController();
}

// Destroys the main application window.
MainWindow::~MainWindow()
{
    delete ui_;
}

// Opens the add test case dialog.
void MainWindow::addTestCase()
{
    if (testing_active_)
        return;

    AddTestCaseDialog dialog(this);

    if (dialog.exec() !=
        QDialog::Accepted)
    {
        return;
    }

    const robot_motion_test_data::TestCase
        test_case =
            dialog.testCase();

    if (!data_access_->createTestCase(
            test_case))
    {
        QMessageBox::warning(
            this,
            "Test Case",
            "Could not create the test case.\n"
            "The name may already exist.");

        return;
    }

    mixed_order_overridden_ = false;

    loadTestCases();
}

// Opens the selected test case for editing.
void MainWindow::editTestCase()
{
    if (testing_active_)
        return;

    const int index =
        selectedTestIndex();

    if (index < 0 ||
        index >=
            static_cast<int>(
                test_cases_.size()))
    {
        return;
    }

    AddTestCaseDialog dialog(
        test_cases_[index],
        this);

    if (dialog.exec() !=
        QDialog::Accepted)
    {
        return;
    }

    const robot_motion_test_data::TestCase
        updated =
            dialog.testCase();

    if (!data_access_->updateTestCase(
            test_cases_[index].name,
            updated))
    {
        QMessageBox::warning(
            this,
            "Test Case",
            "Could not update the test case.");

        return;
    }

    mixed_order_overridden_ = false;

    loadTestCases();
}

// Removes the selected test case.
void MainWindow::removeTestCase()
{
    if (testing_active_)
        return;

    const int index =
        selectedTestIndex();

    if (index < 0 ||
        index >=
            static_cast<int>(
                test_cases_.size()))
    {
        return;
    }

    const QString name =
        QString::fromStdString(
            test_cases_[index].name);

    const QMessageBox::StandardButton
        answer =
            QMessageBox::question(
                this,
                "Delete Test Case",
                "Delete \"" +
                    name +
                    "\"?",
                QMessageBox::Yes |
                QMessageBox::No,
                QMessageBox::No);

    if (answer != QMessageBox::Yes)
        return;

    if (!data_access_->deleteTestCase(
            test_cases_[index].name))
    {
        QMessageBox::warning(
            this,
            "Test Case",
            "Could not delete the test case.");

        return;
    }

    mixed_order_overridden_ = false;

    loadTestCases();
}

// Moves the selected test case upward.
void MainWindow::moveTestCaseUp()
{
    if (testing_active_)
        return;

    const int index =
        selectedTestIndex();

    if (index <= 0)
        return;

    std::swap(
        test_cases_[index],
        test_cases_[index - 1]);

    mixed_order_overridden_ = false;

    updateTestCaseList();

    ui_->testCaseList->setCurrentRow(
        index - 1);

    updateQueueController();
}

// Moves the selected test case downward.
void MainWindow::moveTestCaseDown()
{
    if (testing_active_)
        return;

    const int index =
        selectedTestIndex();

    if (index < 0 ||
        index >=
            static_cast<int>(
                test_cases_.size()) - 1)
    {
        return;
    }

    std::swap(
        test_cases_[index],
        test_cases_[index + 1]);

    mixed_order_overridden_ = false;

    updateTestCaseList();

    ui_->testCaseList->setCurrentRow(
        index + 1);

    updateQueueController();
}

// Starts test execution.
void MainWindow::startTesting()
{
    if (execution_controller_->isRunning())
        return;

    if (test_cases_.empty())
    {
        QMessageBox::warning(
            this,
            "Start Testing",
            "No test cases have been added.");

        return;
    }

    const bool manual_waypoint_mode =
        manual_waypoint_mode_radio_->isChecked();

    QString start_waypoint;
    QString goal_waypoint;

    int manual_start_node_id = -1;
    int manual_goal_node_id = -1;

    if (manual_waypoint_mode)
    {
        bool start_ok = false;
        bool goal_ok = false;

        manual_start_node_id =
            manual_start_vertex_edit_
                ->text()
                .trimmed()
                .toInt(&start_ok);

        manual_goal_node_id =
            manual_goal_vertex_edit_
                ->text()
                .trimmed()
                .toInt(&goal_ok);

        if (!start_ok ||
            !goal_ok ||
            manual_start_node_id < 0 ||
            manual_goal_node_id < 0)
        {
            QMessageBox::warning(
                this,
                "Start Testing",
                "Enter valid start and goal "
                "vertex IDs.");

            return;
        }

        if (manual_start_node_id ==
            manual_goal_node_id)
        {
            QMessageBox::warning(
                this,
                "Start Testing",
                "Start and goal vertex IDs "
                "must be different.");

            return;
        }
    }
    else
    {
        start_waypoint =
            ui_->startWaypointComboBox
                ->currentText()
                .trimmed();

        goal_waypoint =
            ui_->goalWaypointComboBox
                ->currentText()
                .trimmed();

        if (start_waypoint.isEmpty() ||
            goal_waypoint.isEmpty())
        {
            QMessageBox::warning(
                this,
                "Start Testing",
                "Select both start and goal waypoints.");

            return;
        }

        if (start_waypoint ==
            goal_waypoint)
        {
            QMessageBox::warning(
                this,
                "Start Testing",
                "Start and goal waypoints "
                "must be different.");

            return;
        }
    }

    const QMessageBox::StandardButton
        answer =
            QMessageBox::question(
                this,
                "Start Testing",
                "Do you want to continue?",
                QMessageBox::Ok |
                QMessageBox::Cancel,
                QMessageBox::Cancel);

    if (answer != QMessageBox::Ok)
    {
        return;
    }

    /*
     * Clear previous graph data before starting
     * the new test session.
     */
    clearLiveGraphs();

    execution_controller_->setTestCases(
        test_cases_);

    if (manual_waypoint_mode)
    {
        execution_controller_->setManualWaypoints(
            manual_start_node_id,
            manual_goal_node_id);
    }
    else
    {
        execution_controller_->clearManualWaypoints();

        execution_controller_->setWaypoints(
            start_waypoint,
            goal_waypoint);
    }

    testing_active_ = true;

    setTestCaseEditingEnabled(false);

    ui_->startButton->setEnabled(false);

    ui_->stopButton->setEnabled(true);

    updateTestCaseList();

    execution_controller_->start();
}

// Stops test execution.
void MainWindow::stopTesting()
{
    if (!execution_controller_->isRunning())
        return;

    execution_controller_->stop();
}

// Exports the latest saved result for the selected test case.
void MainWindow::exportResults()
{
    if (testing_active_)
    {
        QMessageBox::warning(
            this,
            "Export",
            "Please wait until testing is completed.");
        return;
    }

    const int index =
        selectedTestIndex();

    if (index < 0 ||
        index >=
            static_cast<int>(
                test_cases_.size()))
    {
        QMessageBox::warning(
            this,
            "Export",
            "Please select a test case to export.");
        return;
    }

    const QString test_case_name =
        QString::fromStdString(
            test_cases_[index].name);

    const std::vector<
        robot_motion_test_data::TestResult>
        results =
            data_access_->getAllTestResults();

    bool found = false;

    robot_motion_test_data::TestResult latest_result;

    for (const auto& result : results)
    {
        if (result.test_case_name !=
            test_cases_[index].name)
        {
            continue;
        }

        if (!found ||
            result.executed_at >
                latest_result.executed_at)
        {
            latest_result = result;
            found = true;
        }
    }

    if (!found)
    {
        QMessageBox::warning(
            this,
            "Export",
            "No saved test result was found for \"" +
                test_case_name +
                "\".");
        return;
    }

    const QString export_root =
        "/overlay_ws/exports";

    const QString date_folder =
        QDateTime::currentDateTime()
            .toString("yyyy-MM-dd");

    QString safe_test_case_name =
        test_case_name;

    safe_test_case_name.replace("/", "_");
    safe_test_case_name.replace("\\", "_");
    safe_test_case_name.replace(":", "_");

    const QString export_directory =
        export_root +
        "/" +
        date_folder +
        "/" +
        safe_test_case_name;

    QDir directory;

    if (!directory.mkpath(
            export_directory))
    {
        QMessageBox::critical(
            this,
            "Export",
            "Could not create export directory:\n" +
                export_directory);
        return;
    }

    auto saveGraph =
        [](
            const QString& title,
            const QString& xLabel,
            const QString& yLabel,
            const QVector<QPointF>& data,
            const QString& filePath,
            bool showBrakeMarker,
            double brakeMarkerX,
            const QString& brakeMarkerLabel) -> bool
        {
            if (data.isEmpty())
            {
                return false;
            }

            GraphWidget graph(
                title,
                xLabel,
                yLabel);

            graph.resize(
                1200,
                700);

            graph.setData(data);

            /*
             * The live graphs display the braking event as a red
             * vertical marker. Add the same marker to the temporary
             * export graph before rendering it to PNG.
             */
            if (showBrakeMarker &&
                std::isfinite(brakeMarkerX))
            {
                graph.setEventMarker(
                    brakeMarkerX,
                    brakeMarkerLabel);
            }

            QImage image(
                1200,
                700,
                QImage::Format_ARGB32);

            image.fill(Qt::white);

            graph.render(
                &image,
                QPoint(),
                QRegion(),
                QWidget::DrawChildren);

            return image.save(
                filePath,
                "PNG");
        };

    /*
     * The live application graphs use test-relative coordinates:
     *
     *     plot_time     = raw_time - first_raw_time
     *     plot_distance = raw_distance - first_raw_distance
     *
     * MongoDB stores the raw telemetry coordinates.  For export we
     * must apply the same normalization, otherwise the graph is
     * compressed against the far right side of the PNG.
     */
    double first_time = 0.0;
    double first_distance = 0.0;
    bool have_first_time = false;
    bool have_first_distance = false;

    if (!latest_result.velocity_time.empty())
    {
        first_time =
            latest_result.velocity_time.front().first;
        have_first_time = true;
    }
    else if (!latest_result.acceleration_time.empty())
    {
        first_time =
            latest_result.acceleration_time.front().first;
        have_first_time = true;
    }

    if (!latest_result.velocity_distance.empty())
    {
        first_distance =
            latest_result.velocity_distance.front().first;
        have_first_distance = true;
    }

    QVector<QPointF>
        velocity_time_data;

    for (const auto& sample :
         latest_result.velocity_time)
    {
        const double plot_time =
            have_first_time
                ? sample.first - first_time
                : sample.first;

        velocity_time_data.append(
            QPointF(
                plot_time,
                sample.second));
    }

    QVector<QPointF>
        velocity_distance_data;

    for (const auto& sample :
         latest_result.velocity_distance)
    {
        const double plot_distance =
            have_first_distance
                ? sample.first - first_distance
                : sample.first;

        velocity_distance_data.append(
            QPointF(
                plot_distance,
                sample.second));
    }

    QVector<QPointF>
        acceleration_time_data;

    /*
     * The exported acceleration graph uses the actual acceleration
     * calculated from the saved telemetry, just like the live graph.
     *
     * The acceleration specified in the test case is an upper limit
     * for positive acceleration.  The actual curve is therefore kept,
     * but positive values are capped at that test-case limit.
     *
     * The achieved maximum acceleration itself is still stored
     * separately in latest_result.acceleration_achieved_mps2 and
     * written to the TXT export below.
     */
    const double acceleration_limit =
        test_cases_[index].acceleration_mps2;

    if (!latest_result.acceleration_time.empty())
    {
        bool first_acceleration_sample = true;

        /*
         * Start the exported acceleration graph from zero, matching
         * the live graph behaviour.
         */
        for (const auto& sample :
             latest_result.acceleration_time)
        {
            const double plot_time =
                have_first_time
                    ? sample.first - first_time
                    : sample.first;

            if (first_acceleration_sample)
            {
                acceleration_time_data.append(
                    QPointF(
                        plot_time,
                        0.0));

                first_acceleration_sample = false;
            }

            const double acceleration =
                sample.second;

            const double limited_acceleration =
                std::min(
                    acceleration,
                    acceleration_limit);

            acceleration_time_data.append(
                QPointF(
                    plot_time,
                    limited_acceleration));
        }
    }

    /*
     * Reconstruct the braking marker position for the exported
     * graphs.
     *
     * braking_time_s and braking_distance_m are measured from the
     * exact braking activation point until the robot reaches rest.
     * Therefore the activation point is reconstructed from the final
     * recorded telemetry sample.
     */
    bool export_brake_marker = false;
    double export_brake_time = 0.0;
    double export_brake_distance = 0.0;
    QString export_brake_label;

    if (latest_result.braking_type !=
            robot_motion_test_data::BrakingType::NONE &&
        !latest_result.velocity_time.empty() &&
        !latest_result.velocity_distance.empty() &&
        latest_result.braking_time_s >= 0.0 &&
        latest_result.braking_distance_m >= 0.0)
    {
        const double final_raw_time =
            latest_result.velocity_time.back().first;

        const double final_raw_distance =
            latest_result.velocity_distance.back().first;

        const double raw_brake_time =
            final_raw_time -
            latest_result.braking_time_s;

        const double raw_brake_distance =
            final_raw_distance -
            latest_result.braking_distance_m;

        export_brake_time =
            have_first_time
                ? raw_brake_time - first_time
                : raw_brake_time;

        export_brake_distance =
            have_first_distance
                ? raw_brake_distance - first_distance
                : raw_brake_distance;

        switch (latest_result.braking_type)
        {
            case robot_motion_test_data::BrakingType::STO:
                export_brake_label = "STO ACTIVATED";
                break;

            case robot_motion_test_data::BrakingType::SAFE_STOP:
                export_brake_label = "SAFE-STOP ACTIVATED";
                break;

            case robot_motion_test_data::BrakingType::NORMAL_BRAKING:
                export_brake_label = "NORMAL BRAKING ACTIVATED";
                break;

            case robot_motion_test_data::BrakingType::NONE:
                break;
        }

        export_brake_marker =
            !export_brake_label.isEmpty() &&
            std::isfinite(export_brake_time) &&
            std::isfinite(export_brake_distance);
    }

    if (!saveGraph(
            "Velocity vs Time",
            "Time (s)",
            "Velocity (m/s)",
            velocity_time_data,
            export_directory +
                "/velocity_time.png",
            export_brake_marker,
            export_brake_time,
            export_brake_label))
    {
        QMessageBox::warning(
            this,
            "Export",
            "Could not export velocity_time.png.");
        return;
    }

    if (!saveGraph(
            "Velocity vs Distance",
            "Distance (m)",
            "Velocity (m/s)",
            velocity_distance_data,
            export_directory +
                "/velocity_distance.png",
            export_brake_marker,
            export_brake_distance,
            export_brake_label))
    {
        QMessageBox::warning(
            this,
            "Export",
            "Could not export velocity_distance.png.");
        return;
    }

    if (!acceleration_time_data.isEmpty())
    {
        if (!saveGraph(
                "Acceleration vs Time",
                "Time (s)",
                "Acceleration (m/s²)",
                acceleration_time_data,
                export_directory +
                    "/acceleration_time.png",
                false,
                0.0,
                QString()))
        {
            QMessageBox::warning(
                this,
                "Export",
                "Could not export acceleration_time.png.");
            return;
        }
    }

    const robot_motion_test_data::TestCase&
        test_case =
            test_cases_[index];

    const QString text_file_path =
        export_directory +
        "/test_conditions_and_results.txt";

    QFile text_file(
        text_file_path);

    if (!text_file.open(
            QIODevice::WriteOnly |
            QIODevice::Text))
    {
        QMessageBox::critical(
            this,
            "Export",
            "Could not create:\n" +
                text_file_path);
        return;
    }

    QTextStream stream(
        &text_file);

    QString braking_type;

    switch (latest_result.braking_type)
    {
        case robot_motion_test_data::BrakingType::STO:
            braking_type = "STO";
            break;

        case robot_motion_test_data::BrakingType::SAFE_STOP:
            braking_type = "SAFE_STOP";
            break;

        case robot_motion_test_data::BrakingType::NORMAL_BRAKING:
            braking_type = "NORMAL_BRAKING";
            break;

        case robot_motion_test_data::BrakingType::NONE:
            braking_type = "NONE";
            break;
    }

    QString load_condition;

    switch (latest_result.load_condition)
    {
        case robot_motion_test_data::LoadCondition::NO_LOAD:
            load_condition = "NO_LOAD";
            break;

        case robot_motion_test_data::LoadCondition::UNDER_LOAD:
            load_condition = "UNDER_LOAD";
            break;
    }

    QString status;

    switch (latest_result.status)
    {
        case robot_motion_test_data::TestStatus::COMPLETED:
            status = "COMPLETED";
            break;

        case robot_motion_test_data::TestStatus::STOPPED:
            status = "STOPPED";
            break;
    }

    stream
        << "TEST CONDITIONS AND RESULTS\n"
        << "============================\n\n";

    stream
        << "Test Case Name: "
        << QString::fromStdString(test_case.name)
        << "\n";

    stream
        << "Execution Timestamp: "
        << QString::fromStdString(latest_result.executed_at)
        << "\n\n";

    stream
        << "TEST CONDITIONS\n"
        << "---------------\n";

    stream
        << "Max Velocity Target (m/s): "
        << test_case.max_velocity_mps
        << "\n";

    stream
        << "Acceleration Target (m/s^2): "
        << test_case.acceleration_mps2
        << "\n";

    stream
        << "Braking Type: "
        << braking_type
        << "\n";

    stream
        << "Load Condition: "
        << load_condition
        << "\n";

    stream
        << "Load Mass (kg): "
        << latest_result.load_mass_kg
        << "\n";

    stream
        << "Iterations: "
        << latest_result.iterations_run
        << "\n";

    stream
        << "Start Waypoint ID: "
        << QString::fromStdString(latest_result.start_waypoint_id)
        << "\n";

    stream
        << "Goal Waypoint ID: "
        << QString::fromStdString(latest_result.goal_waypoint_id)
        << "\n\n";

    stream
        << "COMPUTED RESULTS\n"
        << "----------------\n";

    stream
        << "Max Velocity Achieved (m/s): "
        << latest_result.max_velocity_achieved_mps
        << "\n";

    stream
        << "Min Velocity Achieved (m/s): "
        << latest_result.min_velocity_achieved_mps
        << "\n";

    stream
        << "Acceleration Achieved (m/s^2): "
        << latest_result.acceleration_achieved_mps2
        << "\n";

    stream
        << "Braking Distance (m): "
        << latest_result.braking_distance_m
        << "\n";

    stream
        << "Braking Time (s): "
        << latest_result.braking_time_s
        << "\n";

    stream
        << "Status: "
        << status
        << "\n";

    text_file.close();

    QMessageBox::information(
        this,
        "Export Complete",
        "Test result exported successfully.\n\n"
        "Location:\n" +
            export_directory);
}

// Updates the current test information.
void MainWindow::testStarted(
    const QString& name,
    int iteration)
{
    ui_->currentTestLabel->setText(
        "Current Test: " + name);

    ui_->iterationLabel->setText(
        QString("Iteration: %1")
            .arg(iteration));

    ui_->statusLabel->setText(
        "Status: RUNNING");

    /*
     * Keep the live graphs limited to the currently running
     * test case.
     *
     * The controller emits testStarted() for every iteration,
     * so we must only clear the graphs when the test CASE
     * changes, not when another iteration of the same case starts.
     *
     * This does not affect exporting. Export reads the selected
     * test case's saved result from MongoDB, so an older completed
     * test can still be exported even after the live UI graph has
     * moved on to a newer test case.
     */
    int new_test_index = -1;

    for (int i = 0;
         i < static_cast<int>(test_cases_.size());
         ++i)
    {
        if (QString::fromStdString(
                test_cases_[i].name) == name)
        {
            new_test_index = i;
            break;
        }
    }

    if (new_test_index != current_running_test_index_)
    {
        clearLiveGraphs();
    }

    current_running_test_index_ = new_test_index;

    /*
     * Refresh the test-case list so the blue active indicator
     * moves to the test case that has just started.
     */
    updateTestCaseList();
}

void MainWindow::brakeActivated(
    const QString& label,
    double time,
    double distance)
{
    if (!live_telemetry_initialized_)
        return;

    const double plot_time =
        time - live_start_time_;

    const double plot_distance =
        distance - live_start_distance_;

    /*
     * The same brake activation event is displayed on both
     * velocity graphs.
     *
     * Velocity vs Time:
     *     X = telemetry time relative to the test session.
     *
     * Velocity vs Distance:
     *     X = telemetry distance relative to the test session.
     *
     * GraphWidget stores all event markers, so the STO marker is
     * retained when the later Safe-Stop event is published.
     */
    velocity_time_graph_->setEventMarker(
        plot_time,
        label);

    velocity_distance_graph_->setEventMarker(
        plot_distance,
        label);
}

// Handles execution stopping.
void MainWindow::executionStopped()
{
    testing_active_ = false;

    /*
     * No test case is currently running after a stop,
     * so remove the blue active indicator.
     */
    current_running_test_index_ = -1;

    setTestCaseEditingEnabled(true);

    ui_->startButton->setEnabled(true);

    ui_->stopButton->setEnabled(false);

    ui_->statusLabel->setText(
        "Status: STOPPED");

    updateTestCaseList();
}

void MainWindow::executionCompleted()
{
    // Force the final displayed velocity to zero.
    if (!live_velocity_time_data_.isEmpty())
    {
        QPointF last = live_velocity_time_data_.last();
        last.setY(0.0);
        live_velocity_time_data_[live_velocity_time_data_.size() - 1] = last;
    }

    if (!live_velocity_distance_data_.isEmpty())
    {
        QPointF last = live_velocity_distance_data_.last();
        last.setY(0.0);
        live_velocity_distance_data_[live_velocity_distance_data_.size() - 1] = last;
    }

    velocity_time_graph_->setData(
        live_velocity_time_data_);

    velocity_distance_graph_->setData(
        live_velocity_distance_data_);

    testing_active_ = false;
    current_running_test_index_ = -1;

    setTestCaseEditingEnabled(true);

    ui_->startButton->setEnabled(true);

    ui_->stopButton->setEnabled(false);

    ui_->statusLabel->setText(
        "Status: COMPLETED");

    updateTestCaseList();
}

// Clears all live graph data.
void MainWindow::clearLiveGraphs()
{
    live_velocity_time_data_.clear();

    live_velocity_distance_data_.clear();

    live_acceleration_time_data_.clear();

    live_start_time_ = 0.0;

    live_start_distance_ = 0.0;

    live_previous_time_ = 0.0;

    live_previous_velocity_ = 0.0;

    live_telemetry_initialized_ = false;

    velocity_time_graph_->setData(
        live_velocity_time_data_);

    velocity_distance_graph_->setData(
        live_velocity_distance_data_);

    acceleration_time_graph_->setData(
        live_acceleration_time_data_);

    velocity_time_graph_->clearEventMarker();
    velocity_distance_graph_->clearEventMarker();
}

// Updates the graphs and live labels from telemetry.
void MainWindow::updateLiveTelemetry(
    double velocity,
    double distance,
    double time)
{
    /*
     * Only display telemetry as part of an active test.
     */
    if (!testing_active_)
    {
        return;
    }

    /*
     * First telemetry sample establishes the origin
     * for the current test's graphs.
     */
    if (!live_telemetry_initialized_)
    {
        live_start_time_ = time;

        live_start_distance_ = distance;

        live_previous_time_ = time;

        live_previous_velocity_ = velocity;

        live_telemetry_initialized_ = true;
    }

    const double plot_time =
        time - live_start_time_;

    const double plot_distance =
        distance - live_start_distance_;

    live_velocity_time_data_.append(
        QPointF(
            plot_time,
            velocity));

    live_velocity_distance_data_.append(
        QPointF(
            plot_distance,
            velocity));

    /*
     * Calculate acceleration from the actual velocity telemetry,
     * exactly as the original live graph did.
     *
     * The acceleration configured in the current test case is only
     * an upper limit for positive acceleration.  This means the graph
     * can rise and fall naturally, return to zero when the velocity
     * becomes constant, and go negative during deceleration, while
     * never displaying positive acceleration above the test-case limit.
     *
     * The actual achieved acceleration is calculated separately by
     * TestExecutionController and saved to MongoDB.
     */
    if (current_running_test_index_ >= 0 &&
        current_running_test_index_ <
            static_cast<int>(test_cases_.size()))
    {
        const double acceleration_limit =
            test_cases_[current_running_test_index_]
                .acceleration_mps2;

        if (time > live_previous_time_)
        {
            const double dt =
                time - live_previous_time_;

            if (dt > 0.0)
            {
                const double acceleration =
                    (velocity -
                     live_previous_velocity_) /
                    dt;

                const double limited_acceleration =
                    std::min(
                        acceleration,
                        acceleration_limit);

                live_acceleration_time_data_.append(
                    QPointF(
                        plot_time,
                        limited_acceleration));
            }
        }
    }

    live_previous_time_ = time;

    live_previous_velocity_ = velocity;

    velocity_time_graph_->setData(
        live_velocity_time_data_);

    velocity_distance_graph_->setData(
        live_velocity_distance_data_);

    acceleration_time_graph_->setData(
        live_acceleration_time_data_);

    ui_->velocityLabel->setText(
        QString("Velocity: %1 m/s")
            .arg(
                velocity,
                0,
                'f',
                3));

    ui_->distanceLabel->setText(
        QString("Distance: %1 m")
            .arg(
                plot_distance,
                0,
                'f',
                3));
}

// Loads test cases from MongoDB.
void MainWindow::loadTestCases()
{
    if (!data_access_->isConnected())
    {
        test_cases_.clear();

        updateTestCaseList();

        return;
    }

    test_cases_ =
        data_access_->getAllTestCases();

    updateTestCaseList();
}

// Updates the test case list.
void MainWindow::updateTestCaseList()
{
    ui_->testCaseList->clear();

    for (int i = 0;
         i <
         static_cast<int>(
             test_cases_.size());
         ++i)
    {
        QListWidgetItem* item =
            new QListWidgetItem(
                ui_->testCaseList);

        QWidget* row =
            new QWidget;

        QVBoxLayout* row_layout =
            new QVBoxLayout(row);

        row_layout->setContentsMargins(
            0,
            0,
            0,
            0);

        row_layout->setSpacing(0);

        /*
         * A thin blue indicator is shown at the top of
         * the currently running test case only.
         */
        if (i == current_running_test_index_)
        {
            QFrame* active_indicator =
                new QFrame(row);

            active_indicator->setFixedHeight(4);

            active_indicator->setStyleSheet(
                "QFrame {"
                "background-color: #2563eb;"
                "border: none;"
                "}");

            row_layout->addWidget(
                active_indicator);
        }

        QWidget* content_widget =
            new QWidget(row);

        QHBoxLayout* content_layout =
            new QHBoxLayout(
                content_widget);

        content_layout->setContentsMargins(
            12,
            7,
            8,
            7);

        content_layout->setSpacing(6);

        const auto& test_case =
            test_cases_[i];

        QString load_text;

        if (test_case.load_condition ==
            robot_motion_test_data::
                LoadCondition::UNDER_LOAD)
        {
            load_text = "Loaded";
        }
        else
        {
            load_text = "Unloaded";
        }

        QString braking_text;

        switch (test_case.braking_type)
        {
        case robot_motion_test_data::
            BrakingType::STO:
            braking_text = "STO";
            break;

        case robot_motion_test_data::
            BrakingType::SAFE_STOP:
            braking_text = "Safe-Stop";
            break;
        
        
        case robot_motion_test_data::
            BrakingType::NORMAL_BRAKING:
            braking_text = "Normal Braking";
            break;

        case robot_motion_test_data::
            BrakingType::NONE:
            braking_text = "No Braking";
            break;
        }

        QWidget* text_widget =
            new QWidget(row);

        QVBoxLayout* text_layout =
            new QVBoxLayout(text_widget);

        text_layout->setContentsMargins(
            0,
            0,
            0,
            0);

        text_layout->setSpacing(1);

        QLabel* name =
            new QLabel(
                QString::fromStdString(
                    test_case.name));

        name->setWordWrap(true);

        name->setStyleSheet(
            "QLabel {"
            "font-size: 14px;"
            "font-weight: 600;"
            "}");

        QLabel* details =
            new QLabel(
                load_text +
                "    " +
                QString("v = %1")
                    .arg(
                        QString::number(
                            test_case.max_velocity_mps,
                            'g',
                            6)) +
                "    " +
                braking_text);

        details->setWordWrap(true);

        details->setStyleSheet(
            "QLabel {"
            "font-size: 10px;"
            "color: #666666;"
            "}");

        text_layout->addWidget(
            name);

        text_layout->addWidget(
            details);

        // Uses a trash-bin icon for deletion.
        QPushButton* delete_button =
            new QPushButton(
                QString::fromUtf8(
                    "\xF0\x9F\x97\x91"));

        delete_button->setEnabled(
            !testing_active_);

        delete_button->setFixedSize(
            24,
            24);

        delete_button->setToolTip(
            "Delete Test Case");

        delete_button->setStyleSheet(
            "QPushButton {"
            "border: 1px solid palette(mid);"
            "border-radius: 12px;"
            "font-size: 13px;"
            "}"
            "QPushButton:hover {"
            "background: #f0d0d0;"
            "}");

        QPushButton* edit_button =
            new QPushButton("✎");

        edit_button->setEnabled(
            !testing_active_);

        edit_button->setFixedSize(
            24,
            24);

        edit_button->setToolTip(
            "Edit Test Case");

        edit_button->setStyleSheet(
            "QPushButton {"
            "border: 1px solid palette(mid);"
            "border-radius: 12px;"
            "font-size: 16px;"
            "}"
            "QPushButton:hover {"
            "background: palette(alternate-base);"
            "}");

        content_layout->addWidget(
            text_widget,
            1,
            Qt::AlignLeft |
            Qt::AlignVCenter);

        content_layout->addWidget(
            delete_button,
            0,
            Qt::AlignRight |
            Qt::AlignVCenter);

        content_layout->addWidget(
            edit_button,
            0,
            Qt::AlignRight |
            Qt::AlignVCenter);

        QFrame* separator =
            new QFrame(row);

        separator->setFrameShape(
            QFrame::HLine);

        separator->setFrameShadow(
            QFrame::Plain);

        separator->setFixedHeight(1);

        separator->setStyleSheet(
            "QFrame {"
            "color: #d0d0d0;"
            "background-color: #d0d0d0;"
            "}");

        row_layout->addWidget(
            content_widget);

        row_layout->addWidget(
            separator);

        item->setData(
            Qt::UserRole,
            i);

        item->setSizeHint(
            QSize(0, 72));

        ui_->testCaseList->setItemWidget(
            item,
            row);

        connect(
            delete_button,
            &QPushButton::clicked,
            this,
            [this, i]()
            {
                if (i < 0 ||
                    i >=
                        static_cast<int>(
                            test_cases_.size()))
                {
                    return;
                }

                ui_->testCaseList
                    ->setCurrentRow(i);

                removeTestCase();
            });

        connect(
            edit_button,
            &QPushButton::clicked,
            this,
            [this, i]()
            {
                if (i < 0 ||
                    i >=
                        static_cast<int>(
                            test_cases_.size()))
                {
                    return;
                }

                ui_->testCaseList
                    ->setCurrentRow(i);

                editTestCase();
            });
    }

        if (testing_active_ &&
        current_running_test_index_ >= 0 &&
        current_running_test_index_ <
            static_cast<int>(test_cases_.size()))
    {
        ui_->testCaseList->setCurrentRow(
            current_running_test_index_);
    }

    ui_->testCaseCountLabel->setText(
        QString("No: %1")
            .arg(test_cases_.size()));

    showMixedLoadWarningIfNeeded();
}

// Updates the waypoint lists.
void MainWindow::updateWaypointList()
{
    ui_->startWaypointComboBox->clear();

    ui_->goalWaypointComboBox->clear();

    const QStringList waypoint_ids =
        ros_client_->waypointIds();

    ui_->startWaypointComboBox
        ->addItems(waypoint_ids);

    ui_->goalWaypointComboBox
        ->addItems(waypoint_ids);

    ui_->startWaypointComboBox
        ->setCurrentIndex(-1);

    ui_->goalWaypointComboBox
        ->setCurrentIndex(-1);

    showMixedLoadWarningIfNeeded();
}

// Updates the execution queue.
void MainWindow::updateQueueController()
{
    execution_controller_->setTestCases(
        test_cases_);
}

// Displays the mixed-load and route-distance warnings when required.
void MainWindow::showMixedLoadWarningIfNeeded()
{
    clearRightDynamicPanel();

    if (hasMixedLoadOrder() &&
        !mixed_order_overridden_)
    {
        warning_widget_ =
            new QWidget(
                ui_->rightDynamicContainer);

        QVBoxLayout* layout =
            new QVBoxLayout(
                warning_widget_);

        layout->setContentsMargins(
            8,
            12,
            8,
            12);

        layout->setSpacing(8);

        QLabel* icon =
            new QLabel("⚠");

        icon->setAlignment(
            Qt::AlignCenter);

        icon->setStyleSheet(
            "font-size: 28px;"
            "color: #d9a300;");

        QLabel* text =
            new QLabel(
                "Unloaded conditions have "
                "to be done first.");

        text->setAlignment(
            Qt::AlignCenter);

        text->setWordWrap(true);

        QPushButton* override_button =
            new QPushButton(
                "OVERRIDE");

        override_button->setFixedHeight(
            32);

        layout->addWidget(icon);

        layout->addWidget(text);

        layout->addWidget(
            override_button);

        ui_->rightDynamicLayout
            ->addWidget(
                warning_widget_);

        connect(
            override_button,
            &QPushButton::clicked,
            this,
            [this]()
            {
                mixed_order_overridden_ = true;

                showMixedLoadWarningIfNeeded();
            });
    }

    showDistanceWarningIfNeeded();
}

// Displays a non-blocking warning when the selected graph route is too
// short to reach the configured maximum velocity.
void MainWindow::showDistanceWarningIfNeeded()
{
    const bool manual_waypoint_mode =
        manual_waypoint_mode_radio_ &&
        manual_waypoint_mode_radio_->isChecked();

    double route_distance = -1.0;

    if (manual_waypoint_mode)
    {
        bool start_ok = false;
        bool goal_ok = false;

        const int start_node_id =
            manual_start_vertex_edit_->text()
                .trimmed()
                .toInt(&start_ok);

        const int goal_node_id =
            manual_goal_vertex_edit_->text()
                .trimmed()
                .toInt(&goal_ok);

        if (!start_ok ||
            !goal_ok ||
            start_node_id < 0 ||
            goal_node_id < 0 ||
            start_node_id == goal_node_id)
        {
            return;
        }

        route_distance =
            ros_client_->routeDistanceByNodeId(
                start_node_id,
                goal_node_id);
    }
    else
    {
        const QString start_waypoint =
            ui_->startWaypointComboBox
                ->currentText()
                .trimmed();

        const QString goal_waypoint =
            ui_->goalWaypointComboBox
                ->currentText()
                .trimmed();

        if (start_waypoint.isEmpty() ||
            goal_waypoint.isEmpty() ||
            start_waypoint == goal_waypoint)
        {
            return;
        }

        route_distance =
            ros_client_->routeDistance(
                start_waypoint,
                goal_waypoint);
    }

    if (!std::isfinite(route_distance) ||
        route_distance < 0.0)
    {
        return;
    }

    QStringList warnings;

    for (const auto& test_case :
         test_cases_)
    {
        const double max_velocity =
            test_case.max_velocity_mps;

        const double acceleration =
            test_case.acceleration_mps2;

        if (!std::isfinite(max_velocity) ||
            !std::isfinite(acceleration) ||
            max_velocity <= 0.0 ||
            acceleration <= 0.0)
        {
            continue;
        }

        const double minimum_distance =
            (max_velocity * max_velocity) /
            (2.0 * acceleration);

        if (route_distance + 1e-9 <
            minimum_distance)
        {
            warnings.append(
                QString("%1: at least %2 m "
                        "to reach %3 m/s")
                    .arg(
                        QString::fromStdString(
                            test_case.name))
                    .arg(
                        minimum_distance,
                        0,
                        'f',
                        2)
                    .arg(
                        max_velocity,
                        0,
                        'f',
                        2));
        }
    }

    if (warnings.isEmpty())
    {
        return;
    }

    QWidget* distance_warning_widget =
        new QWidget(
            ui_->rightDynamicContainer);

    QVBoxLayout* layout =
        new QVBoxLayout(
            distance_warning_widget);

    layout->setContentsMargins(
        8,
        8,
        8,
        12);

    layout->setSpacing(6);

    QLabel* icon =
        new QLabel("⚠");

    icon->setAlignment(
        Qt::AlignCenter);

    icon->setStyleSheet(
        "font-size: 28px;"
        "color: #d9a300;");

    QLabel* title =
        new QLabel(
            "The selected route is too short "
            "to reach the configured velocity.");

    title->setAlignment(
        Qt::AlignCenter);

    title->setWordWrap(true);

    QLabel* available =
        new QLabel(
            QString("Available route distance: %1 m")
                .arg(
                    route_distance,
                    0,
                    'f',
                    2));

    available->setAlignment(
        Qt::AlignCenter);

    available->setWordWrap(true);

    QLabel* required =
        new QLabel(
            "Minimum distance required:\n" +
            warnings.join("\n"));

    required->setAlignment(
        Qt::AlignCenter);

    required->setWordWrap(true);

    layout->addWidget(icon);
    layout->addWidget(title);
    layout->addWidget(available);
    layout->addWidget(required);

    ui_->rightDynamicLayout
        ->addWidget(
            distance_warning_widget);
}

// Determines whether an unloaded case appears after a loaded case.
bool MainWindow::hasMixedLoadOrder() const
{
    bool seen_under_load = false;

    for (const auto& test_case :
         test_cases_)
    {
        if (test_case.load_condition ==
            robot_motion_test_data::
                LoadCondition::UNDER_LOAD)
        {
            seen_under_load = true;
        }
        else if (seen_under_load)
        {
            return true;
        }
    }

    return false;
}

// Removes conditional content from the right sidebar.
void MainWindow::clearRightDynamicPanel()
{
    while (QLayoutItem* item =
               ui_->rightDynamicLayout
                   ->takeAt(0))
    {
        if (QWidget* widget =
                item->widget())
        {
            widget->deleteLater();
        }

        delete item;
    }

    warning_widget_ = nullptr;
}

// Enables or disables test-case editing controls.
void MainWindow::setTestCaseEditingEnabled(
    bool enabled)
{
    ui_->addTestCaseButton->setEnabled(
        enabled);
}

// Returns the selected test case index.
int MainWindow::selectedTestIndex() const
{
    QListWidgetItem* item =
        ui_->testCaseList->currentItem();

    if (!item)
        return -1;

    const int index =
        item->data(
            Qt::UserRole).toInt();

    if (index < 0 ||
        index >=
            static_cast<int>(
                test_cases_.size()))
    {
        return -1;
    }

    return index;
}