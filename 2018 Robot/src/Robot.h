#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <iostream>
#include <LiveWindow/LiveWindow.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>
#include <PID/ElevatorPIDHelper.h>

#include "Constants.h"
#include "PID/AnglePIDOutput.h"
#include "PID/DistancePIDHelper.h"
#include "Sensors/AngleSensorGroup.h"

using namespace frc;
using namespace std;

inline double PulsesToInches(double sensorPosition)
{
	double circumference = consts::WHEEL_DIAMETER * consts::PI;
	double revolutions = sensorPosition / consts::PULSES_PER_REV;
	double distance = revolutions * circumference;

	return distance;
}

class Robot: public TimedRobot
{
private:
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
	Spark FrontRightMotor;
	Spark FrontLeftMotor;
	WPI_TalonSRX BackLeftMotor;
	WPI_TalonSRX RightIntakeMotor;
	WPI_TalonSRX ElevatorMotor;
	WPI_TalonSRX LeftIntakeMotor;
	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	XboxController DriveController;
	DifferentialDrive DriveTrain;
	AngleSensorGroup AngleSensors;

	AnglePIDOutput AnglePIDOut;
	DistancePIDHelper DistancePID;
	PIDController AngleController;
	PIDController MaintainAngleController;
	PIDController DistanceController;

	ElevatorPIDHelper ElevatorPID;
	PIDController ElevatorPIDController;

	SendableChooser<consts::AutoPosition> *AutoLocationChooser;
	SendableChooser<consts::AutoObjective> *AutoObjectiveChooser;
	SendableChooser<consts::SwitchApproach> *SwitchApproachChooser;

public:
	// Here, we're declaring the following functions:
	// - Robot class constructor
	// - Virtual functions from TimedRobot
	// - Driving and turning functions for autonomous
	// - Pathing functions for autonomous
	// - Testing functions for PID tuning

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
	void TestPeriodic() override;\

	void DriveFor(double seconds, double speed);
	void DriveDistance(double distance);
	void TurnAngle(double angle);

	// Autonomous Robot Functionality
	void DriveToBaseline();
	void SidePath(consts::AutoPosition start, char switchPosition, char scalePosition);
	void OppositeSwitch(consts::AutoPosition start);
	void OppositeScale(consts::AutoPosition start);
	void MiddlePath(char switchPosition);
	void DropCube(double driveSetpoint, consts::ElevatorIncrement elevatorSetpoint);
	void EjectCube();
	void RaiseElevator(consts::ElevatorIncrement elevatorSetpoint);

	// PID Tuning Functions (JUST FOR TESTING)
	void MaintainHeadingTest();
	void DriveDistanceTest(double distance);
	void TurnAngleTest(double angle);

	void ResetEncoders();
};

#endif /* ROBOT */
