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
	SmartDashboard::PutBoolean("Linkage", 0);
	SmartDashboard::PutBoolean("Intake", 0);
	SmartDashboard::PutBoolean("Climb", 0);

	SmartDashboard::PutBoolean("Toggle Elevator Safety", 0);
}

void Robot::DisabledPeriodic()
{
//	std::string AutoCheck = "";
//
//	switch(AutoLocationCHooser->GetSelected())
//	{
//	case consts::AutoPosition::LEFT_START:
//		break;
//	case consts::AutoPosition::RIGHT_START:
//		break;
//	case consts::AutoPosition::MIDDLE_START:
//		break;
//	}
//	AutoCheck += AutoLocationChooser->GetSelected();
//	AutoCheck += AutoObjectiveChooser->GetSelected();
//	AutoCheck += SwitchApproachChooser->GetSelected();
//
//	SmartDashboard::PutString("Auto Settings", AutoCheck);
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
	LinkageMotor.Set(0);
	RightIntakeMotor.Set(0);
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	LeftIntakeMotor.Set(0);
}
