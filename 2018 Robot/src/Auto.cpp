#include "Robot.h"
#include "auto/AnglePIDOutput.h"

void Robot::AutonomousInit()
{
	TurnAngle(90);
}

void Robot::AutonomousPeriodic()
{

}

void Robot::TurnAngle(double angle)
{
	Gyro.Reset();

	AnglePIDOutput angleOutput = AnglePIDOutput(DriveTrain);
	PIDController angleController(0.03, 0, 0, 0, Gyro, angleOutput);
	angleController.SetSetpoint(angle);
	angleController.SetPercentTolerance(1);

	angleController.Enable();
}
