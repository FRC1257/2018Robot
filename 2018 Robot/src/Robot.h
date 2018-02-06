#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <LiveWindow/LiveWindow.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>

#include "Constants.h"
#include "auto/AnglePIDOutput.h"
#include "auto/DistancePIDHelper.h"
#include "auto/AngleSensorGroup.h"

using namespace frc;

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
	// - 3 PIDControllers to manage turning to specific angles, driving specific distances, and maintaining a specific angle

	// - 2 SendableChoosers for selecting an autonomous mode

	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX BackLeftMotor;
	WPI_TalonSRX BackRightMotor;
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

	SendableChooser<consts::AutoPosition> *AutoLocationChooser;
	SendableChooser<consts::AutoObjective> *AutoObjectiveChooser;

public:
	// Here, we're declaring the following functions:
	// - Robot class constructor
	// - Virtual functions from TimedRobot
	// - Driving and turning functions for autonomous
	// - Testing functions for PID tuning

	Robot();
	void RobotInit() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestInit() override;
	void TestPeriodic() override;

	void DriveFor(double seconds, double speed);
	void DriveForward(double distance);
	void TurnAngle(double angle);

	void ToSwitch(char position);
	void ToScale(char position);

	// PID Tuning Functions (JUST FOR TESTING)
	void MaintainHeadingTest();
	void DriveDistanceTest(double distance);

	void ResetEncoders();
};



#endif /* ROBOT */
