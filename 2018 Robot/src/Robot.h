#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <ctre/Phoenix.h>
using namespace frc;

class Robot: public TimedRobot
{
private:
	// Here's a breakdown of what member objects we're declaring:
	// - 4 Motor Controllers for each of the drive motors (WPI_TalonSRX)
	// - 1 Xbox Controller for controlling the robot
	// - 1 DifferentialDrive object to access ArcadeDrive
	// - 2 SpeedControllerGroups to contain the left and right side motors

	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX BackLeftMotor;
	WPI_TalonSRX BackRightMotor;
	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	XboxController DriveController;
	DifferentialDrive DriveTrain;

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
};



#endif /* ROBOT */
