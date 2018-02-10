#include "Robot.h"

Robot::Robot() :
	BackRightMotor(1),
	FrontRightMotor(2),
	FrontLeftMotor(3),
	BackLeftMotor(4),
	LinkageMotor(5),
	RightIntakeMotor(6),
	ClimbMotor(7),
	ElevatorMotor(8),
	LeftIntakeMotor(9),
	IntakeUltrasonic(1, 0),
	ElevatorEncoder(0, 1, false, Encoder::EncodingType::k4X), // TODO: delete or replace aChannel and bChannel
	ElevatorPID(0.25, 0, 0, ElevatorEncoder, ElevatorMotor),
	LinkagePID(0.25, 0, 0, ElevatorEncoder, LinkageMotor),// TODO: replace PID values
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	OperatorController(1),
	DriveTrain(LeftMotors, RightMotors),
	m_isLowering(false),
	m_targetStep(0)
{

}


void Robot::RobotInit()
{
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	RightIntakeMotor.Set(0);
	LeftIntakeMotor.Set(0);
	LinkageMotor.SetNeutralMode(Brake);
	IntakeUltrasonic.SetAutomaticMode(true);

	// Current limiting
	FrontLeftMotor.ConfigContinuousCurrentLimit(consts::FOURTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	FrontLeftMotor.EnableCurrentLimit(true);

	FrontRightMotor.ConfigContinuousCurrentLimit(consts::FOURTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	FrontRightMotor.EnableCurrentLimit(true);

	BackLeftMotor.ConfigContinuousCurrentLimit(consts::FOURTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	BackLeftMotor.EnableCurrentLimit(true);

	BackRightMotor.ConfigContinuousCurrentLimit(consts::FOURTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	BackRightMotor.EnableCurrentLimit(true);

	RightIntakeMotor.ConfigContinuousCurrentLimit(consts::FOURTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	RightIntakeMotor.EnableCurrentLimit(true);

	LeftIntakeMotor.ConfigContinuousCurrentLimit(consts::THIRTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	LeftIntakeMotor.EnableCurrentLimit(true);
}

START_ROBOT_CLASS(Robot)
