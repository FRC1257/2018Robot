#include "Robot.h"

void Robot::TestInit()
{

}

void Robot::TestPeriodic()
{
	SmartDashboard::PutNumber("Auto Pos Val", (int) AutoLocationChooser->GetSelected());
	SmartDashboard::PutNumber("Auto Obj Val", (int) AutoObjectiveChooser->GetSelected());
}

void Robot::DriveDistanceTest(double distance)
{

}
