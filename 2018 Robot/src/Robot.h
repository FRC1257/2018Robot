#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Encoder.h>
#include <IterativeRobot.h>
#include <PID/TalonPIDHelper.h>
#include "Constants.h"

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
	TalonPIDHelper ElevatorEncoderSource;
	TalonPIDHelper LinkageEncoderSource;
	PIDController ElevatorPIDController;
	PIDController LinkagePIDController;
	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	XboxController DriveController;
	XboxController OperatorController;
	DifferentialDrive DriveTrain;

	// Keep track of the state of the Elevator PID
	bool m_isLowering;
	int m_targetStep;

public:
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

	void Drive();
	void Elevator();
	void Climb();
	void Intake();
	void Linkage();

	double GetClosestStepNumber();

	void DriveTest();
	void FullElevatorTest();
	void ManualElevatorTest();
	void LinkageTest();
	void IntakeTest();
	void ClimbTest();
	void CurrentTest();
};

#endif /* ROBOT */
