#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <LiveWindow/LiveWindow.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <string.h>
#include <sstream>
#include <vector>
#include <sys/types.h>
#include <dirent.h>

#include "Constants.h"
#include "PID/AnglePIDOutput.h"
#include "PID/DistancePIDHelper.h"
#include "Sensors/AngleSensorGroup.h"

using namespace frc;

inline double PulsesToInches(double sensorPosition)
{
	double circumference = consts::WHEEL_DIAMETER * consts::PI;
	double revolutions = sensorPosition / consts::PULSES_PER_REV;
	double distance = revolutions * circumference;

	return distance;
}

inline double dabs(double value)
{
	return value > 0 ? value : -value;
}

inline double deadband(double value)
{
	return dabs(value) < 0.02 ? 0 : value;
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
	// - 3 PIDControllers to manage turning to specific angles, driving specific distances, and maintaining a specific angle

	// - 3 SendableChoosers for selecting an autonomous mode

	// - 1 ofstream for logging motor outputs
	// - 1 ifstream for taking in motor outputs

	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX FrontRightMotor;
	Spark BackLeftMotor;
	Spark BackRightMotor;
	WPI_TalonSRX ElevatorMotor;
	WPI_TalonSRX RightIntakeMotor;
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

	Encoder ElevatorEncoder;
	PIDController ElevatorPID;

	SendableChooser<consts::AutoPosition> *AutoLocationChooser;
	SendableChooser<consts::AutoObjective> *AutoObjectiveChooser;
	SendableChooser<consts::MiddleApproach> *MiddleApproachChooser;
	SendableChooser<std::string> *EchoAutoFileNameChooser;

	std::ofstream outf;
	std::ifstream inf;

public:
	// Here, we're declaring the following functions:
	// - Robot class constructor
	// - Virtual functions from TimedRobot
	// - Driving and turning functions for autonomous
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
	void DriveForward(double distance);
	void TurnAngle(double angle);
	void SidePath(consts::AutoPosition start, char switchPosition, char scalePosition);
	void MiddlePath(char switchPosition);

	// PID Tuning Functions (JUST FOR TESTING)
	void MaintainHeadingTest();
	void DriveDistanceTest(double distance);
	void TurnAngleTest(double angle);

	void LogMotorOutput();
	void ReadLog();

	void ResetEncoders();
};



#endif /* ROBOT */
