#ifndef ROBOT
#define ROBOT

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <Encoder.h>
#include <IterativeRobot.h>
#include <PID/ElevatorPIDHelper.h>
#include <Sensors/StabilizedUltrasonic.h>
#include "Constants.h"

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
	StabilizedUltrasonic IntakeUltrasonic;
	ElevatorPIDHelper ElevatorPID;
	PIDController ElevatorPIDController;
	SpeedControllerGroup LeftMotors;
	SpeedControllerGroup RightMotors;
	XboxController DriveController;
	XboxController OperatorController;
	DifferentialDrive DriveTrain;

	// Keep track of the state of the Elevator PID
	bool m_isElevatorLowering;
	bool m_isElevatorInAutoMode;
	int m_targetElevatorStep;

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
	double CapElevatorOutput(double output);
	void CapElevatorSetpoint(double& setpoint);

	void DriveTest();
	void FullElevatorTest();
	void PIDElevatorTest();
	void ManualElevatorTest();
	void LinkageTest();
	void IntakeTest();
	void ClimbTest();
	void CurrentTest();
	void RunMotorsTestFor(int numberOfSeconds);
};

#endif /* ROBOT */
