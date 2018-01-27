#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <AHRS.h>
#include "auto/AnglePIDOutput.h"

using namespace frc;

#define PI 3.1416
#define WHEEL_DIAMETER 6
#define PULSES_PER_REV 4096

double PulsesToInches(double SensorPosition)
{
	double circumference = WHEEL_DIAMETER * PI;
	double revolutions = SensorPosition / PULSES_PER_REV;
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

	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX BackLeftMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX BackRightMotor;
	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	XboxController DriveController;
	DifferentialDrive DriveTrain;
//	ADXRS450_Gyro Gyro;
	AHRS NavX;

	AnglePIDOutput AnglePID;
	PIDController AngleController;

public:
	// Here, we're declaring the following functions:
	// - Robot class constructor
	// - Virtual functions from TimedRobot

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

	void DriveFor(double distance, double speed);
	void TurnAngle(double angle);
};



#endif /* ROBOT */
