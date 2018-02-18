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
	IntakeUltrasonic(1, 0), // TODO check port #
	ElevatorEncoder(0, 1, false, Encoder::EncodingType::k4X), // TODO: delete or replace aChannel and bChannel
	ElevatorPID(0.25, 0, 0, ElevatorEncoder, ElevatorMotor),
	LinkagePID(0.25, 0, 0, ElevatorEncoder, LinkageMotor),// TODO: replace PID values
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	OperatorController(1),
	DriveTrain(LeftMotors, RightMotors)
{
	m_isLowering = false;
	m_inAutomatic = false;
	m_targetStep = 0;

}


void Robot::RobotInit()
{
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	RightIntakeMotor.Set(0);
	LeftIntakeMotor.Set(0);
	LinkageMotor.SetNeutralMode(Brake);

	// Talon Configuration for Velocity Closed Loop Drive
	FrontLeftMotor.ConfigNominalOutputForward(0, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontLeftMotor.ConfigNominalOutputReverse(0, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontLeftMotor.ConfigPeakOutputForward(1, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontLeftMotor.ConfigPeakOutputReverse(-1, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontLeftMotor.Config_kF(0, 0.1097, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontLeftMotor.Config_kP(0, 0.22, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontLeftMotor.Config_kI(0, 0.0, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontLeftMotor.Config_kD(0, 0.0, consts::TALON_FUNCTION_TIMEOUT_MS);

	FrontRightMotor.ConfigNominalOutputForward(0, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontRightMotor.ConfigNominalOutputReverse(0, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontRightMotor.ConfigPeakOutputForward(1, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontRightMotor.ConfigPeakOutputReverse(-1, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontRightMotor.Config_kF(0, 0.1097, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontRightMotor.Config_kP(0, 0.22, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontRightMotor.Config_kI(0, 0.0, consts::TALON_FUNCTION_TIMEOUT_MS);
	FrontRightMotor.Config_kD(0, 0.0, consts::TALON_FUNCTION_TIMEOUT_MS);

}

START_ROBOT_CLASS(Robot)
