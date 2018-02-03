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
	AngleSensors(SPI::Port::kMXP, SPI::kOnboardCS0),
	AnglePIDOut(DriveTrain),
	DistancePID(FrontLeftMotor, DriveTrain),
	AngleController(0.0111, 0, 0, 0, AngleSensors, AnglePIDOut),
	MaintainAngleController(0.01, 0, 0, AngleSensors, AnglePIDOut),
	DistanceController(0.05, 0, 0, DistancePID, DistancePID)
{
	AutoLocationChooser = new SendableChooser<constants::AutoPosition>();
	AutoObjectiveChooser = new SendableChooser<constants::AutoObjective>();
	lw = LiveWindow::GetInstance();
}


void Robot::RobotInit()
{
	// Configuring the Talon Drive Encoders
	FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
			constants::kPIDLoopIdx, constants::kTimeoutMs);
	FrontLeftMotor.SetSensorPhase(true);
	FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,
			constants::kPIDLoopIdx, constants::kTimeoutMs);
	FrontRightMotor.SetSensorPhase(true);

	// Adding PID Controllers to LiveWindow
	lw->Add(&AngleController);
	lw->Add(&MaintainAngleController);
	lw->Add(&DistanceController);
}



START_ROBOT_CLASS(Robot)
