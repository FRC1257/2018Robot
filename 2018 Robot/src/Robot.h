#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <ctre/Phoenix.h>

#include "Constants.h"
#include <cmath>
using namespace frc;
using namespace std;

inline double dabs(double d) { return d > 0.0 ? d : -d; } // Absolute value of a double precision floating point number
inline bool inDeadZone(double axisVal) { return dabs(axisVal) < 0.2; }
inline double Limit(double motorValue)
{
	if(motorValue > 1.)
	{
		return 1.;
	}
	if(motorValue < -1.)
	{
		return -1.;
	}
	else
	{
		return motorValue;
	}
}

class Robot: public TimedRobot
{
private:
	// Here's a breakdown of what member objects we're declaring:
	// - 4 Motor Controllers for each of the drive motors (WPI_TalonSRX)
	// - 1 Xbox Controller for controlling the robot
	// - 1 DifferentialDrive object to access ArcadeDrive
	// - 2 SpeedControllerGroups to contain the left and right side motors

	WPI_TalonSRX BackRightMotor;
	WPI_TalonSRX FrontRightMotor;
	WPI_TalonSRX FrontLeftMotor;
	WPI_TalonSRX BackLeftMotor;
	WPI_TalonSRX LinkageMotor;
	WPI_TalonSRX RightIntakeMotor;
	WPI_TalonSRX ClimbMotor;
	WPI_TalonSRX ElevatorMotor;
	WPI_TalonSRX LeftIntakeMotor;
	Ultrasonic IntakeUltrasonic;
	Encoder ElevatorEncoder;
	PIDController ElevatorPID;
	PIDController LinkagePID;
	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	XboxController DriveController;
	XboxController OperatorController;
	DifferentialDrive DriveTrain;

	bool m_isLowering;
	bool m_inAutomatic;
	int m_targetStep;

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
	double GetStepNumber(double elevatorHeight);
	void DriveTest();
	void ElevatorTest();
	void LinkageTest();
	void IntakeTest();
	void ClimbTest();
	void Drive();
	void Elevator();
	void Climb();
	void Intake();
	void Linkage();
	void VelocityArcadeDrive(double forwardSpeed, double turnSpeed, bool squaredInputs);
};


#endif /* ROBOT */
