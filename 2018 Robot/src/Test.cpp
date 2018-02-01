#include "Robot.h"

void Robot::TestInit()
{
	TurnAngle(90);
}

void Robot::TestPeriodic()
{
	SmartDashboard::PutNumber("Gyro", Gyro.GetAngle());
}
