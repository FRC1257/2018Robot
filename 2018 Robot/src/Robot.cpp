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
	AngleSensor(SPI::Port::kMXP, SPI::kOnboardCS0),
	AnglePIDOut(DriveTrain),
	DistancePID(FrontLeftMotor, DriveTrain),
	AngleController(0.0111, 0, 0, 0, AngleSensor, AnglePIDOut),
	MaintainAngleController(0.01, 0, 0, AngleSensor, AnglePIDOut),
	DistanceController(0.05, 0, 0, DistancePID, DistancePID)
{
	AutoLocationChooser = new SendableChooser<constants::AutoPosition>();
	AutoObjectiveChooser = new SendableChooser<constants::AutoObjective>();
}


void Robot::RobotInit()
{
	FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
			constants::kPIDLoopIdx, constants::kTimeoutMs);
	FrontLeftMotor.SetSensorPhase(true);

	FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
			constants::kPIDLoopIdx, constants::kTimeoutMs);
	FrontRightMotor.SetSensorPhase(true);

	BackLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
			constants::kPIDLoopIdx, constants::kTimeoutMs);
	BackLeftMotor.SetSensorPhase(true);

	BackRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
			constants::kPIDLoopIdx, constants::kTimeoutMs);
	BackRightMotor.SetSensorPhase(true);
}



START_ROBOT_CLASS(Robot)
