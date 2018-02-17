#include "Robot.h"

Robot::Robot() :
	FrontLeftMotor(1),
	FrontRightMotor(2),
	BackLeftMotor(3),
	BackRightMotor(4),
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
	AngleController(0.0111, 0, 0, 0, AngleSensors, AnglePIDOut),
	MaintainAngleController(0.01, 0, 0, AngleSensors, AnglePIDOut),
	DistanceController(0.05, 0, 0, DistancePID, DistancePID),

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
	AngleController.SetAbsoluteTolerance(0.5);
	AngleController.SetOutputRange(-1.0, 1.0);

	// Configuring Maintain Angle PID Controller
	MaintainAngleController.SetAbsoluteTolerance(0.5);
	MaintainAngleController.SetOutputRange(-1.0, 1.0);

	// Configuring Distance PID Controller
	DistanceController.SetPercentTolerance(1);
	DistanceController.SetOutputRange(-1.0, 1.0);
}

void Robot::ResetEncoders()
{
	FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
	FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
}


START_ROBOT_CLASS(Robot)
