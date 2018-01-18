#include "Robot.h"


Robot::Robot() :
	FrontLeftMotor(1),
	BackLeftMotor(2),
	FrontRightMotor(3),
	BackRightMotor(4),
	ElevatorMotor (6), //TODO Check port values
	IntakeMotor (7),   //TODO Check port values
	ClimbMotor (8),    //TODO Check port values
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	OperatorController(1),
	DriveTrain(LeftMotors, RightMotors)

{

}


void Robot::RobotInit()
{
	elevatorEncoder(0, 1, false, FeedbackDevice::CTRE_MagEncoder_Relative); // TODO: delete or replace aChannel and bChannel
	elevatorPID(1, 0, 0, FeedbackDevice::CTRE_MagEncoder_Relative, ElevatorMotor); // TODO: replace PID values
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	IntakeMotor.Set(0);
}



START_ROBOT_CLASS(Robot)
