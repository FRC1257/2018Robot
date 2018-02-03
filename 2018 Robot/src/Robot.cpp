#include "Robot.h"

Robot::Robot() :
	FrontLeftMotor(1),
	FrontRightMotor(2),
	BackLeftMotor(3),
	BackRightMotor(4),
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	DriveTrain(LeftMotors, RightMotors),
	Gyro(SPI::kOnboardCS0),
	AnglePIDOut(DriveTrain),
	DistancePID(FrontLeftMotor, DriveTrain),
	AngleController(0.0111, 0, 0, 0, Gyro, AnglePIDOut),
	MaintainAngleController(0.025, 0, 0, Gyro, AnglePIDOut),
	DistanceController(0.03, 0, 0, DistancePID, DistancePID)
{
	AutoLocationChooser = new SendableChooser<constants::AutoPosition>();
	AutoObjectiveChooser = new SendableChooser<constants::AutoObjective>();
}


void Robot::RobotInit()
{
	int kPIDLoopIdx = 0;
	int kTimeoutMs = 10;

	FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
	FrontLeftMotor.SetSensorPhase(true);

	FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
	FrontRightMotor.SetSensorPhase(true);

	BackLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
	BackLeftMotor.SetSensorPhase(true);

	BackRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);
	BackRightMotor.SetSensorPhase(true);
}



START_ROBOT_CLASS(Robot)
