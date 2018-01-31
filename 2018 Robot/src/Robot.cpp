#include "Robot.h"

Robot::Robot() :
	FrontLeftMotor(1),
	BackLeftMotor(2),
	FrontRightMotor(3),
	BackRightMotor(4),
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	DriveTrain(LeftMotors, RightMotors),
	Gyro(SPI::kOnboardCS0),
	AnglePID(DriveTrain),
	DistancePID(FrontLeftMotor, DriveTrain), //FrontLeft is placeholder until we learn which motor has an encoder
	AngleController(0.609, 0, 0, 0, Gyro, AnglePID), // P value = 0.609
	DistanceController(0.01, 0, 0, DistancePID, DistancePID)
{

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
