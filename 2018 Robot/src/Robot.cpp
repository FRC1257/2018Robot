#include "Robot.h"


Robot::Robot() :
	FrontLeftMotor(1),
	FrontRightMotor(2),
	BackLeftMotor(3),
	BackRightMotor(4),
	ElevatorMotor (5), //TODO Check port values
	RightIntakeMotor (6), //TODO Check port values
	LeftIntakeMotor (7),  //TODO Check port values
	LinkageMotor (8),  //TODO Check port values
	ClimbMotor (9),    //TODO Check port values
	elevatorEncoder(0, 1, false, Encoder::EncodingType::k4X), // TODO: delete or replace aChannel and bChannel
	elevatorPID(1, 0, 0, elevatorEncoder, ElevatorMotor),
	linkagePID(1, 0, 0, elevatorEncoder, LinkageMotor),// TODO: replace PID values
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
	RightIntakeMotor.Set(0);
	LeftIntakeMotor.Set(0);
	LinkageMotor.SetNeutralMode(Brake);
}



START_ROBOT_CLASS(Robot)
