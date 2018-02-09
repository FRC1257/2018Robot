#include "Robot.h"

Robot::Robot() :
	FrontLeftMotor(1),
	FrontRightMotor(2),
	BackLeftMotor(0), //0
	BackRightMotor(1), //1
	ElevatorMotor(5),
	RightIntakeMotor(6),
	LeftIntakeMotor(7),
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	DriveTrain(LeftMotors, RightMotors),
	AngleSensors(SPI::Port::kMXP, SPI::kOnboardCS0),
	AnglePIDOut(DriveTrain),
	DistancePID(FrontLeftMotor, DriveTrain),
	AngleController(0.042, 0.01, 0.1, 0, AngleSensors, AnglePIDOut),
	MaintainAngleController(0.01, 0, 0, AngleSensors, AnglePIDOut),
	DistanceController(0.05, 0, 0, DistancePID, DistancePID),
	ElevatorEncoder(0, 1, false, Encoder::EncodingType::k4X), // TODO: delete or replace aChannel and bChannel
	ElevatorPID(0.25, 0, 0, ElevatorEncoder, ElevatorMotor)
{
	AutoLocationChooser = new SendableChooser<consts::AutoPosition>();
	AutoObjectiveChooser = new SendableChooser<consts::AutoObjective>();
}


void Robot::RobotInit()
{
	// Configuring the Talon Drive Encoders
	FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
			consts::PIDLoopIdx, consts::timeoutMs);
	FrontLeftMotor.SetSensorPhase(true);
	FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
			consts::PIDLoopIdx, consts::timeoutMs);
	FrontRightMotor.SetSensorPhase(true);

	// Adding PID Controllers to LiveWindow
	LiveWindow::GetInstance()->Add(&AngleController);
	LiveWindow::GetInstance()->Add(&MaintainAngleController);
	LiveWindow::GetInstance()->Add(&DistanceController);
}

void Robot::ResetEncoders()
{
	FrontLeftMotor.SetSelectedSensorPosition(0, consts::PIDLoopIdx, consts::timeoutMs);
	FrontRightMotor.SetSelectedSensorPosition(0, consts::PIDLoopIdx, consts::timeoutMs);
}


START_ROBOT_CLASS(Robot)
