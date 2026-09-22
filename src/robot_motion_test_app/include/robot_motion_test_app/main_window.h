#ifndef ROBOT_MOTION_TEST_APP_MAIN_WINDOW_H
#define ROBOT_MOTION_TEST_APP_MAIN_WINDOW_H

#include <QMainWindow>
#include <QPointF>
#include <QVector>
#include <QWidget>

#include <memory>
#include <vector>

#include "robot_motion_test_data/models.h"
#include "robot_motion_test_data/data_access_layer.h"

class GraphWidget;
class RosClient;
class TestExecutionController;
class QLineEdit;
class QRadioButton;

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void addTestCase();
    void editTestCase();
    void removeTestCase();
    void moveTestCaseUp();
    void moveTestCaseDown();
    void startTesting();
    void stopTesting();
    void testStarted(const QString& name, int iteration);
    void executionStopped();
    void exportResults();
    void brakeActivated(
        const QString& label,
        double time,
        double distance);

private:
    void loadTestCases();
    void updateTestCaseList();
    void updateWaypointList();
    void updateQueueController();
    void showMixedLoadWarningIfNeeded();
    void clearRightDynamicPanel();
    void executionCompleted();
    void setTestCaseEditingEnabled(bool enabled);
    bool hasMixedLoadOrder() const;
    int selectedTestIndex() const;
    void updateLiveTelemetry(double velocity, double distance, double time);
    void clearLiveGraphs();

    Ui::MainWindow* ui_;
    std::unique_ptr<robot_motion_test_data::DataAccessLayer> data_access_;
    std::unique_ptr<RosClient> ros_client_;
    std::unique_ptr<TestExecutionController> execution_controller_;
    std::vector<robot_motion_test_data::TestCase> test_cases_;

    GraphWidget* velocity_time_graph_;
    GraphWidget* velocity_distance_graph_;
    GraphWidget* acceleration_time_graph_;

    QVector<QPointF> live_velocity_time_data_;
    QVector<QPointF> live_velocity_distance_data_;
    QVector<QPointF> live_acceleration_time_data_;

    double live_start_time_;
    double live_start_distance_;
    double live_previous_time_;
    double live_previous_velocity_;
    bool live_telemetry_initialized_;

    QWidget* warning_widget_;
    bool mixed_order_overridden_;
    bool testing_active_;
    int current_running_test_index_ = -1;

    // ---------------------------------------------------------
    // Manual vertex mode
    // ---------------------------------------------------------

    QRadioButton* graph_waypoint_mode_radio_;
    QRadioButton* manual_waypoint_mode_radio_;

    QLineEdit* manual_start_vertex_edit_;
    QLineEdit* manual_goal_vertex_edit_;
};

#endif