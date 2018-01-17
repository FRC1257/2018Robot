#include "Robot.h"

Robot::Robot() :
	FrontLeftMotor(1),
	BackLeftMotor(2),
	FrontRightMotor(3),
	BackRightMotor(4),
	ElevatorMotor (6), //Check port values
	IntakeMotor (7),   //Check port values
	ClimbMotor (8),    //Check port values
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	OperatorController(1),
	DriveTrain(LeftMotors, RightMotors)
{

}


void Robot::RobotInit()
{
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	IntakeMotor.Set(0);
}



START_ROBOT_CLASS(Robot)
