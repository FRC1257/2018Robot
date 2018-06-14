#include "Robot.h"

void Robot::DisabledInit()
{
	StopCurrentProcesses();

	// Autonomous Modes
	SmartDashboard::PutBoolean("Test Angle", 0);
	SmartDashboard::PutBoolean("Test Maintain", 0);
	SmartDashboard::PutBoolean("Test Distance", 0);

	// Sensor Resets
	SmartDashboard::PutBoolean("Reset Angle", 0);
	SmartDashboard::PutBoolean("Reset Encoders", 0);

	// Maintain Angle Test buttons/output
	SmartDashboard::PutNumber("Test Maintain Output", 0);
	SmartDashboard::PutBoolean("Enable Test Distance Output", 0);
	SmartDashboard::PutBoolean("Enable Maintain Controller", 0);
	SmartDashboard::PutBoolean("Toggle Maintain Test", 0);

	// Distance Tests
	SmartDashboard::PutBoolean("Go Forward, Turn Right", 0);
	SmartDashboard::PutBoolean("Toggle Distance Test", 0);

	// Auto Elevator Tests
	SmartDashboard::PutBoolean("Test Auto Elevator", 0);
	SmartDashboard::PutNumber("Desired Increment", 0);
	SmartDashboard::PutBoolean("Go to Increment", 0);

	// SmartDashboard code to toggle each teleop test function
	SmartDashboard::PutBoolean("Drive", 0);
	SmartDashboard::PutBoolean("Full Elevator", 0);
	SmartDashboard::PutBoolean("Manual Elevator", 0);
	SmartDashboard::PutBoolean("PID Elevator", 0);
	SmartDashboard::PutBoolean("Intake", 0);

	SmartDashboard::PutBoolean("Toggle Elevator Safety", 0);
}

void Robot::DisabledPeriodic()
{
	std::string AutoCheck = "";

	switch(AutoLocationChooser->GetSelected())
	{
	case consts::AutoPosition::LEFT_START:
		AutoCheck += "Left";
		break;
	case consts::AutoPosition::RIGHT_START:
		AutoCheck += "Right";
		break;
	case consts::AutoPosition::MIDDLE_START:
		AutoCheck += "Middle";
		break;
	}

	AutoCheck += ", ";

	switch(AutoObjectiveChooser->GetSelected())
	{
	case consts::AutoObjective::BASELINE:
		AutoCheck += "Baseline";
		break;
	case consts::AutoObjective::DEFAULT:
		AutoCheck += "Default Path";
		break;
	case consts::AutoObjective::SCALE:
		AutoCheck += "Scale";
		break;
	case consts::AutoObjective::SWITCH:
		AutoCheck += "Switch";
		break;
	}

	AutoCheck += ", Switch Approach: ";

	switch(SwitchApproachChooser->GetSelected())
	{
	case consts::SwitchApproach::FRONT:
		AutoCheck += "Front";
		break;
	case consts::SwitchApproach::SIDE:
		AutoCheck += "Side";
		break;
	}

	SmartDashboard::PutString("Auto Settings", AutoCheck);
}

void Robot::StopCurrentProcesses()
{
	ResetSensors();
	DisablePIDControllers();
	ZeroMotors();
}

void Robot::ResetSensors()
{
	ResetDriveEncoders();
	AngleSensors.Reset();
}

void Robot::ResetDriveEncoders()
{
	FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
	FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
}

void Robot::DisablePIDControllers()
{
	AngleController.Disable();
	MaintainAngleController.Disable();
	DistanceController.Disable();
	ElevatorPIDController.Disable();
}

void Robot::ZeroMotors()
{
	BackRightMotor.Set(0);
	FrontRightMotor.Set(0);
	FrontLeftMotor.Set(0);
	BackLeftMotor.Set(0);
	RightIntakeMotor.Set(0);
	RightElevatorMotor.Set(0);
	LeftElevatorMotor.Set(0);
	LeftIntakeMotor.Set(0);
}
