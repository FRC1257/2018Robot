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
	LeftMotors(FrontLeftMotor, BackLeftMotor),
	RightMotors(FrontRightMotor, BackRightMotor),
	DriveTrain(LeftMotors, RightMotors),
	DriveController(0),
	OperatorController(1),
	IntakeUltrasonic(1, 0),
	AngleSensors(SPI::Port::kMXP, SPI::kOnboardCS0),

	ElevatorPID(&ElevatorMotor),
	AnglePIDOut(DriveTrain),
	DistancePID(FrontLeftMotor, DriveTrain),
	AngleController(0.035, 0.0075, 0.095, AngleSensors, AnglePIDOut), //(0.02525, 0, 0.025)
	MaintainAngleController(0.02, 0.01, 0.11, AngleSensors, AnglePIDOut), //(0.03, 0.0015, 0.06)
	DistanceController(0.0225, 0, 0.005, DistancePID, DistancePID), //(0.04, 0, 0)
	ElevatorPIDController(0.25, 0., 0., ElevatorPID, ElevatorPID),
	m_isElevatorLowering(false),
	m_isElevatorInAutoMode(false),
	m_targetElevatorStep(0),

	echoAutoPathFileOut(),
	echoAutoPathFileIn(),
	m_finishedEcho(false)
{
	AutoLocationChooser = new SendableChooser<consts::AutoPosition>();
	AutoObjectiveChooser = new SendableChooser<consts::AutoObjective>();
	SwitchApproachChooser = new SendableChooser<consts::SwitchApproach>();
	EchoAutoFileNameChooser = new SendableChooser<std::string>();
}

Robot::~Robot()
{
	delete AutoLocationChooser;
	delete AutoObjectiveChooser;
	delete SwitchApproachChooser;
	delete EchoAutoFileNameChooser;
}

void Robot::RobotInit()
{
	StopCurrentProcesses();
	LinkageMotor.SetNeutralMode(Brake);
	ElevatorMotor.SetNeutralMode(Brake);

	// Current limiting
	FrontLeftMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	FrontLeftMotor.EnableCurrentLimit(true);

	FrontRightMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	FrontRightMotor.EnableCurrentLimit(true);

	BackLeftMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	BackLeftMotor.EnableCurrentLimit(true);

	BackRightMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	BackRightMotor.EnableCurrentLimit(true);

	RightIntakeMotor.ConfigContinuousCurrentLimit(consts::FORTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	RightIntakeMotor.EnableCurrentLimit(true);

	LeftIntakeMotor.ConfigContinuousCurrentLimit(consts::THIRTY_AMP_FUSE_CONT_MAX, consts::CONT_CURRENT_TIMEOUT_MS);
	LeftIntakeMotor.EnableCurrentLimit(true);

	ElevatorMotor.ConfigContinuousCurrentLimit(consts::ELEVATOR_CONT_CURRENT_MAX, consts::ELEVATOR_CONT_CURRENT_TIMEOUT_MS);
	ElevatorMotor.EnableCurrentLimit(true);

	// Linkage and Elevator Setup
	LinkageMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, consts::PID_LOOP_X, consts::TIMEOUT_MS);
	LinkageMotor.SetSensorPhase(true);

	ElevatorMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, consts::PID_LOOP_X, consts::TIMEOUT_MS);
	ElevatorMotor.SetSensorPhase(true);

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
	AngleController.SetOutputRange(-0.75, 0.75);

	// Configuring Maintain Angle PID Controller
	MaintainAngleController.SetAbsoluteTolerance(0.5);
	MaintainAngleController.SetOutputRange(-1.0, 1.0);

	// Configuring Distance PID Controller
	DistanceController.SetAbsoluteTolerance(5);
	DistanceController.SetOutputRange(-0.80, 0.80);

	// Setup camera stream in a separate thread
	std::thread visionThread(VisionThread);
	visionThread.detach();
}

START_ROBOT_CLASS(Robot)
