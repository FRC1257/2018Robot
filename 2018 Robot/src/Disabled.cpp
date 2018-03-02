#include "Robot.h"

void Robot::DisabledInit()
{
	StopCurrentProcesses();
}

void Robot::DisabledPeriodic()
{

}

void Robot::StopCurrentProcesses()
{
	ResetSensors();
	DisablePIDControllers();
	ZeroMotors();
}

void Robot::ResetSensors()
{

}

void Robot::DisablePIDControllers()
{
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
