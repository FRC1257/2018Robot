#include "Robot.h"

Robot::Robot() :
	FrontLeftMotor(1),
	FrontRightMotor(2),
	BackLeftMotor(3),
	BackRightMotor(4),
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	DriveTrain(LeftMotors, RightMotors)
{

}


void Robot::RobotInit()
{

}



START_ROBOT_CLASS(Robot)
