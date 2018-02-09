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
	IntakeUltrasonic.SetAutomaticMode(true);

	FrontLeftMotor.ConfigContinuousCurrentLimit(CONTINUOUSAMPS, TIMEOUTMS);
	FrontLeftMotor.ConfigPeakCurrentLimit(PEAKAMPS, TIMEOUTMS);
	FrontLeftMotor.ConfigPeakCurrentDuration(DURATIONSMS, TIMEOUTMS);
	FrontLeftMotor.EnableCurrentLimit(true);

	FrontRightMotor.ConfigContinuousCurrentLimit(CONTINUOUSAMPS, TIMEOUTMS);
	FrontRightMotor.ConfigPeakCurrentLimit(PEAKAMPS, TIMEOUTMS);
	FrontRightMotor.ConfigPeakCurrentDuration(DURATIONSMS, TIMEOUTMS);
	FrontRightMotor.EnableCurrentLimit(true);

	BackLeftMotor.ConfigContinuousCurrentLimit(CONTINUOUSAMPS, TIMEOUTMS);
	BackLeftMotor.ConfigPeakCurrentLimit(PEAKAMPS, TIMEOUTMS);
	BackLeftMotor.ConfigPeakCurrentDuration(DURATIONSMS, TIMEOUTMS);
	BackLeftMotor.EnableCurrentLimit(true);

	BackRightMotor.ConfigContinuousCurrentLimit(CONTINUOUSAMPS, TIMEOUTMS);
	BackRightMotor.ConfigPeakCurrentLimit(PEAKAMPS, TIMEOUTMS);
	BackRightMotor.ConfigPeakCurrentDuration(DURATIONSMS, TIMEOUTMS);
	BackRightMotor.EnableCurrentLimit(true);

	RightIntakeMotor.ConfigContinuousCurrentLimit(CONTINUOUSAMPS, TIMEOUTMS);
	RightIntakeMotor.ConfigPeakCurrentLimit(PEAKAMPS, TIMEOUTMS);
	RightIntakeMotor.ConfigPeakCurrentDuration(DURATIONSMS, TIMEOUTMS);
	RightIntakeMotor.EnableCurrentLimit(true);

	LeftIntakeMotor.ConfigContinuousCurrentLimit(CONTINUOUSAMPS, TIMEOUTMS);
	LeftIntakeMotor.ConfigPeakCurrentLimit(PEAKAMPS, TIMEOUTMS);
	LeftIntakeMotor.ConfigPeakCurrentDuration(DURATIONSMS, TIMEOUTMS);
	LeftIntakeMotor.EnableCurrentLimit(true);
}

START_ROBOT_CLASS(Robot)
