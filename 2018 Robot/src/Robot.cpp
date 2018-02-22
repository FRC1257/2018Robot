#include "Robot.h"

Robot::Robot() :
	BackRightMotor(1),
	FrontRightMotor(2),
	FrontLeftMotor(1),
	BackLeftMotor(0),
//	LinkageMotor(5),
	RightIntakeMotor(6),
//	ClimbMotor(7),
	ElevatorMotor(8),
	LeftIntakeMotor(9),
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveController(0),
	DriveTrain(LeftMotors, RightMotors),
	AngleSensors(SPI::Port::kMXP, SPI::kOnboardCS0),

	AnglePIDOut(DriveTrain),
	DistancePID(FrontLeftMotor, DriveTrain),
	AngleController(0.035, 0.0075, 0.095, AngleSensors, AnglePIDOut),
	MaintainAngleController(0.02, 0.01, 0.11, AngleSensors, AnglePIDOut),
	DistanceController(0.0225, 0, 0.005, DistancePID, DistancePID),

	ElevatorPID(&ElevatorMotor),
	ElevatorPIDController(0.25, 0, 0, ElevatorPID, ElevatorPID)
{
	AutoLocationChooser = new SendableChooser<consts::AutoPosition>();
	AutoObjectiveChooser = new SendableChooser<consts::AutoObjective>();
	SwitchApproachChooser = new SendableChooser<consts::SwitchApproach>();
}

Robot::~Robot()
{
	delete AutoLocationChooser;
	delete AutoObjectiveChooser;
	delete SwitchApproachChooser;
}

void Robot::RobotInit()
{
	// Configuring the Talon Drive Encoders
	FrontLeftMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
	FrontLeftMotor.SetSensorPhase(true);
	FrontRightMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
	FrontRightMotor.SetSensorPhase(true);

	// Adding PID Controllers to LiveWindow
	LiveWindow::GetInstance()->Add(&AngleController);
	LiveWindow::GetInstance()->Add(&MaintainAngleController);
	LiveWindow::GetInstance()->Add(&DistanceController);

	// Configuring Angle PID Controller
	AngleController.SetAbsoluteTolerance(1);
	AngleController.SetOutputRange(-1.0, 1.0);

	// Configuring Maintain Angle PID Controller
	MaintainAngleController.SetAbsoluteTolerance(0.5);
	MaintainAngleController.SetOutputRange(-1.0, 1.0);

	// Configuring Distance PID Controller
	DistanceController.SetPercentTolerance(20);
	DistanceController.SetOutputRange(-0.85, 0.85);
}

void Robot::ResetEncoders()
{
	FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
	FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
}


START_ROBOT_CLASS(Robot)
