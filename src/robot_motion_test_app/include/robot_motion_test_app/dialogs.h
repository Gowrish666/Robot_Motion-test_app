#ifndef ROBOT_MOTION_TEST_APP_DIALOGS_H
#define ROBOT_MOTION_TEST_APP_DIALOGS_H

#include <QDialog>

#include "robot_motion_test_data/models.h"

namespace Ui
{
class AddTestCase;
class LoadPrompt;
}

class AddTestCaseDialog : public QDialog
{
    Q_OBJECT

public:

    explicit AddTestCaseDialog(
        QWidget* parent = nullptr);

    explicit AddTestCaseDialog(
        const robot_motion_test_data::TestCase& testCase,
        QWidget* parent = nullptr);

    ~AddTestCaseDialog();

    robot_motion_test_data::TestCase testCase() const;

private slots:

    void onLoadConditionChanged(int index);

private:

    void loadTestCase(
        const robot_motion_test_data::TestCase& testCase);

    bool validate();

    Ui::AddTestCase* ui_;
};

class LoadPromptDialog : public QDialog
{
    Q_OBJECT

public:

    LoadPromptDialog(
        double massKg,
        QWidget* parent = nullptr);

    ~LoadPromptDialog();

private:

    Ui::LoadPrompt* ui_;
};

#endif