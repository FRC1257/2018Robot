#include "Robot.h"

Robot::Robot() :
	FrontLeftMotor(1),
	BackLeftMotor(2),
	FrontRightMotor(3),
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
