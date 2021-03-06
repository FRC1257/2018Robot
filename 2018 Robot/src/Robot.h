#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <LiveWindow/LiveWindow.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>

#include "Constants.h"
#include <PID/AnglePIDOutput.h>
#include <PID/ElevatorPIDHelper.h>
#include <PID/DistancePIDHelper.h>
#include <Sensors/AngleSensorGroup.h>
#include <Sensors/StabilizedUltrasonic.h>

using namespace frc;

inline double PulsesToInches(double sensorPosition)
{
	double circumference = consts::WHEEL_DIAMETER * consts::PI;
	double revolutions = sensorPosition / consts::PULSES_PER_REV;
	double distance = revolutions * circumference;

	return distance;
}

// Absolute value of a double precision floating point number
inline double dabs(double d) { return d > 0.0 ? d : -d; }
inline double applyDeadband(double axisVal) { return dabs(axisVal) < 0.08 ? 0 : axisVal; }
inline double limit(double d, double cap = 1) {
	if(d > cap)      return cap;
	else if(d < -cap) return -cap;
	else             return d;
}

class Robot: public TimedRobot
{
private:
	// TODO: Update the following list after merging
	// Here's a breakdown of what member objects we're declaring:
	// - 4 Motor Controllers for each of the drive motors (WPI_TalonSRX)
	// - 2 SpeedControllerGroups to contain the left and right side motors
	// - 1 Xbox Controller for controlling the robot
	// - 1 DifferentialDrive object to access ArcadeDrive
	// - 1 AngleSensorGroup for sensing the angle and motion of the robot
	// 		- Internally contains one NavX (AHRS) and one ADXRS450_Gyro

	// - 1 AnglePIDOutput to send turning motor output to DifferentialDrive during PID turning
	// - 1 DistancePIDHelper to manage the source and motor output of DifferentialDrive during PID driving
	// - 1 TalonPIDHelper to manage the source and motor output for the elevator motor
	// - 4 PIDControllers to manage turning to angles, driving distances, maintaining an angle, and raising the elevator

	// - 3 SendableChoosers for selecting an autonomous mode

	WPI_TalonSRX BackRightMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX BackLeftMotor;
	WPI_TalonSRX RightIntakeMotor;
	WPI_TalonSRX LeftIntakeMotor;
	WPI_TalonSRX RightElevatorMotor;
	WPI_TalonSRX LeftElevatorMotor;
	WPI_TalonSRX LinkageMotor;
	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	DifferentialDrive DriveTrain;
	XboxController DriveController;
	XboxController OperatorController;
	AngleSensorGroup AngleSensors;

	DoubleSolenoid LeftSolenoid;
	DoubleSolenoid RightSolenoid;

	ElevatorPIDHelper ElevatorPID;
	AnglePIDOutput AnglePIDOut;
	DistancePIDHelper DistancePID;
	PIDController AngleController;
	PIDController MaintainAngleController;
	PIDController DistanceController;
	PIDController ElevatorPIDController;

	SendableChooser<consts::AutoPosition> *AutoLocationChooser;
	SendableChooser<consts::AutoObjective> *AutoObjectiveChooser;
	SendableChooser<consts::SwitchApproach> *SwitchApproachChooser;

	// Member variables to keep track of the state of the Elevator PID
	bool m_isElevatorLowering;
	bool m_isElevatorInAutoMode;
	int m_targetElevatorStep;
	Timer EjectTimer;

public:
	// Constructor and virtual functions
	Robot();
	~Robot();
	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestInit() override;
	void TestPeriodic() override;

	// Autonomous Robot Functionality
	void DriveFor(double seconds, double speed = 0.5);
	void DriveDistance(double distance, double timeout = consts::PID_TIMEOUT_S);
	void TurnAngle(double angle, double timeout = consts::PID_TIMEOUT_S);
	void DriveToBaseline();
	void SidePath(consts::AutoPosition start, char switchPosition, char scalePosition);
	void OppositeSwitch(consts::AutoPosition start);
	void OppositeScale(consts::AutoPosition start);
	void MiddlePath(char switchPosition);
	void DropCube(consts::ElevatorIncrement elevatorSetpoint);
	void EjectCube(double intakeSpeed = consts::INTAKE_SPEED);
	void RaiseElevator(consts::ElevatorIncrement elevatorSetpoint, double timeout = consts::PID_TIMEOUT_S);

	// Camera Stream code
	static void VisionThread();

	// Code to kill current processes between robot loops
	void StopCurrentProcesses();
	void ResetSensors();
	void ResetDriveEncoders();
	void DisablePIDControllers();
	void ZeroMotors();

	// Teleoperated helper functions
	void Drive();
	void Elevator();
	void ManualElevator();
	void Intake();
	void Linkage();

	// Safety Functions
	bool IsElevatorTooHigh();

	// Automatic elevator functionality
	double GetClosestStepNumber();
	double CapElevatorOutput(double output, bool safetyModeEnabled = false);
	void CapElevatorSetpoint(double& setpoint);

	// Modular test code functions
	void TeleopTest();
	void AutonomousTest();
	void DriveTest();
	void FullElevatorTest();
	void PIDElevatorTest();
	void ManualElevatorTest();
	void AutoElevatorTest();
	void IntakeTest();
	void LinkageTest();
	void CurrentTest();
	void RunMotorsTestFor(int numberOfSeconds);

	// PID Tuning Functions (JUST FOR TESTING)
	void MaintainHeadingTest();
	void DriveDistanceTest(double distance);
	void TurnAngleTest(double angle);
};

#endif /* ROBOT */
