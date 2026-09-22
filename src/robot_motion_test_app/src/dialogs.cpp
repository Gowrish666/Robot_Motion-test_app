#include "robot_motion_test_app/dialogs.h"

#include "ui_add_test_case.h"
#include "ui_load_prompt.h"

#include <QMessageBox>


AddTestCaseDialog::AddTestCaseDialog(
    QWidget* parent)
    : QDialog(parent),
      ui_(new Ui::AddTestCase)
{
    ui_->setupUi(this);

    connect(
        ui_->loadConditionComboBox,
        QOverload<int>::of(
            &QComboBox::currentIndexChanged),
        this,
        &AddTestCaseDialog::onLoadConditionChanged);

    onLoadConditionChanged(
        ui_->loadConditionComboBox->currentIndex());
}


AddTestCaseDialog::AddTestCaseDialog(
    const robot_motion_test_data::TestCase& testCase,
    QWidget* parent)
    : AddTestCaseDialog(parent)
{
    loadTestCase(testCase);
}


AddTestCaseDialog::~AddTestCaseDialog()
{
    delete ui_;
}


robot_motion_test_data::TestCase
AddTestCaseDialog::testCase() const
{
    robot_motion_test_data::TestCase testCase;

    testCase.name =
        ui_->nameLineEdit->text()
            .trimmed()
            .toStdString();

    testCase.max_velocity_mps =
        ui_->maxVelocityDoubleSpinBox->value();

    testCase.acceleration_mps2 =
        ui_->accelerationDoubleSpinBox->value();

    testCase.iterations =
        ui_->iterationsSpinBox->value();


    switch (
        ui_->brakingTypeComboBox->currentIndex())
    {
        case 0:
            testCase.braking_type =
                robot_motion_test_data::BrakingType::STO;
            break;

        case 1:
            testCase.braking_type =
                robot_motion_test_data::BrakingType::SAFE_STOP;
            break;

        case 2:
            testCase.braking_type =
                robot_motion_test_data::BrakingType::NORMAL_BRAKING;
            break;

        default:
            testCase.braking_type =
                robot_motion_test_data::BrakingType::NONE;
            break;
    }


    if (
        ui_->loadConditionComboBox->currentIndex()
        == 0)
    {
        testCase.load_condition =
            robot_motion_test_data::LoadCondition::NO_LOAD;

        testCase.load_mass_kg = 0.0;
    }
    else
    {
        testCase.load_condition =
            robot_motion_test_data::LoadCondition::UNDER_LOAD;

        testCase.load_mass_kg =
            ui_->loadMassDoubleSpinBox->value();
    }

    return testCase;
}


void AddTestCaseDialog::loadTestCase(
    const robot_motion_test_data::TestCase& testCase)
{
    ui_->nameLineEdit->setText(
        QString::fromStdString(testCase.name));

    ui_->maxVelocityDoubleSpinBox->setValue(
        testCase.max_velocity_mps);

    ui_->accelerationDoubleSpinBox->setValue(
        testCase.acceleration_mps2);

    ui_->iterationsSpinBox->setValue(
        testCase.iterations);


    switch (testCase.braking_type)
    {
        case robot_motion_test_data::BrakingType::STO:

            ui_->brakingTypeComboBox
                ->setCurrentIndex(0);

            break;


        case robot_motion_test_data::BrakingType::SAFE_STOP:

            ui_->brakingTypeComboBox
                ->setCurrentIndex(1);

            break;


        case robot_motion_test_data::BrakingType::NORMAL_BRAKING:

            ui_->brakingTypeComboBox
                ->setCurrentIndex(2);

            break;


        case robot_motion_test_data::BrakingType::NONE:

            ui_->brakingTypeComboBox
                ->setCurrentIndex(3);

            break;
    }


    switch (testCase.load_condition)
    {
        case robot_motion_test_data::LoadCondition::NO_LOAD:

            ui_->loadConditionComboBox
                ->setCurrentIndex(0);

            break;


        case robot_motion_test_data::LoadCondition::UNDER_LOAD:

            ui_->loadConditionComboBox
                ->setCurrentIndex(1);

            break;
    }


    ui_->loadMassDoubleSpinBox->setValue(
        testCase.load_mass_kg);
}


void AddTestCaseDialog::onLoadConditionChanged(
    int index)
{
    const bool loaded =
        index == 1;

    ui_->loadMassDoubleSpinBox
        ->setEnabled(loaded);
}


bool AddTestCaseDialog::validate()
{
    if (
        ui_->nameLineEdit->text()
            .trimmed()
            .isEmpty())
    {
        QMessageBox::warning(
            this,
            "Invalid Test Case",
            "Test case name cannot be empty.");

        return false;
    }


    if (
        ui_->maxVelocityDoubleSpinBox->value()
        <= 0.0)
    {
        QMessageBox::warning(
            this,
            "Invalid Test Case",
            "Maximum velocity must be greater than zero.");

        return false;
    }


    if (
        ui_->accelerationDoubleSpinBox->value()
        <= 0.0)
    {
        QMessageBox::warning(
            this,
            "Invalid Test Case",
            "Acceleration must be greater than zero.");

        return false;
    }


    if (
        ui_->loadConditionComboBox->currentIndex()
        == 1 &&
        ui_->loadMassDoubleSpinBox->value()
        <= 0.0)
    {
        QMessageBox::warning(
            this,
            "Invalid Test Case",
            "Load mass must be greater than zero.");

        return false;
    }


    if (
        ui_->iterationsSpinBox->value()
        <= 0)
    {
        QMessageBox::warning(
            this,
            "Invalid Test Case",
            "Iterations must be greater than zero.");

        return false;
    }

    return true;
}


LoadPromptDialog::LoadPromptDialog(
    double massKg,
    QWidget* parent)
    : QDialog(parent),
      ui_(new Ui::LoadPrompt)
{
    ui_->setupUi(this);

    ui_->massLabel->setText(
        QString(
            "Load mass: %1 kg")
            .arg(massKg));
}


LoadPromptDialog::~LoadPromptDialog()
{
    delete ui_;
}