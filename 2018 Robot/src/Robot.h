#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include "auto/AnglePIDOutput.h"
#include "auto/DistancePIDHelper.h"

using namespace frc;

#define PI 3.1416
#define WHEEL_DIAMETER 6
#define PULSES_PER_REV 4096

inline double PulsesToInches(double sensorPosition)
{
	double circumference = WHEEL_DIAMETER * PI;
	double revolutions = sensorPosition / PULSES_PER_REV;
	double distance = revolutions * circumference;

	return distance;
}

class Robot: public TimedRobot
{
private:
	// Here's a breakdown of what member objects we're declaring:
	// - 4 Motor Controllers for each of the drive motors (WPI_TalonSRX)
	// - 1 Xbox Controller for controlling the robot
	// - 1 DifferentialDrive object to access ArcadeDrive
	// - 2 SpeedControllerGroups to contain the left and right side motors
	// - 1 AHRS for sensing the angle and motion of the robot

	// - 1 AnglePIDOutput to send turning motor output to DifferentialDrive during PID turning
	// - 1 DistancePIDHelper to manage the source and motor output of DifferentialDrive during PID driving
	// - 3 PIDControllers to manage turning to specific angles, driving specific distances, and maintaining a specific angle

	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX BackLeftMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX BackRightMotor;
	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	XboxController DriveController;
	DifferentialDrive DriveTrain;
	ADXRS450_Gyro Gyro;

	AnglePIDOutput AnglePID;
	DistancePIDHelper DistancePID;
	PIDController AngleController;
	PIDController MaintainAngleController;
	PIDController DistanceController;

public:
	// Here, we're declaring the following functions:
	// - Robot class constructor
	// - Virtual functions from TimedRobot
	// - Driving and turning functions for autonomous

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

//	void DriveFor(double distance, double speed);
	void DriveForward(double distance);
	void TurnAngle(double angle);
};



#endif /* ROBOT */
