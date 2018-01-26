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
	NavX.Reset();

	angleController.Reset();
	angleController.SetSetpoint(angle);
	angleController.SetPercentTolerance(5);

	angleController.Enable();
}
