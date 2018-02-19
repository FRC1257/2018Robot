#include "Robot.h"

double Robot::GetClosestStepNumber()
{
	for(int i = 0; i < 5; i++)
	{
		// If the elevator is directly below a given setpoint, go to that setpoint
		if(ElevatorPID.GetHeightInches() < consts::ELEVATOR_SETPOINTS[i])
		{
			return i;
		}
	}
	return 4;
}

//Driver Controls
void Robot::Drive()
{
	double forwardSpeed = 0;
	double turnSpeed = 0;

	// If they press A, use single stick arcade with the left joystick
	if(DriveController.GetAButton())
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kLeftHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kLeftHand);
	}
	// If they press the left bumper, use the left joystick for forward and
	// backward motion and the right joystick for turning
	else if(DriveController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kLeftHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kRightHand);
	}
	// If they press the right bumper, use the right joystick for forward and
	// backward motion and the left joystick for turning
	else if(DriveController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		forwardSpeed = DriveController.GetY(GenericHID::JoystickHand::kRightHand);
		turnSpeed = DriveController.GetX(GenericHID::JoystickHand::kLeftHand);
	}

	// Negative is used to make forward positive and backwards negative
	// because the y-axes of the XboxController are natively inverted
	DriveTrain.ArcadeDrive(-forwardSpeed, turnSpeed);
}

// Operator Controls
void Robot::Elevator()
{
	// Use the right trigger to manually raise the elevator and
	// the left trigger to lower the elevator
	double raiseElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kRightHand));
	double lowerElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kLeftHand));

	// If either triggers are being pressed, disable the PID and
	// set the motor to the given speed
	if(raiseElevatorOutput != 0.0 || lowerElevatorOutput != 0.0)
	{
		ElevatorPIDController.Disable();
		ElevatorMotor.Set(raiseElevatorOutput - lowerElevatorOutput);
		return;
	}

	// Automatic Mode is controlled by both bumpers
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		// If elevator is lowering and the right bumper is pressed, stop elevator where it is
		if (m_isLowering)
		{
			ElevatorMotor.Set(0);
			m_isLowering = false;
			ElevatorPIDController.Disable();
		}
		else
		{
			// If right bumper is being pressed for the first time, increase the desired preset by 1
			if (!ElevatorPIDController.IsEnabled())
			{
				m_targetStep = GetClosestStepNumber();
			}
			// If right bumper has already been pressed, go to the next step.
			else if (m_targetStep < 4)
			{
				m_targetStep++;
			}
			ElevatorPIDController.SetSetpoint(consts::ELEVATOR_SETPOINTS[m_targetStep]);
			ElevatorPIDController.Enable();
			m_isLowering = false;
		}
	}
	// The left bumper will lower the elevator to the bottom
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		m_isLowering = true;
		ElevatorPIDController.SetSetpoint(0);
		ElevatorPIDController.Enable();
	}
}

void Robot::Intake()
{
	// Use the B button to intake
	if(OperatorController.GetBButton() && IntakeUltrasonic.GetRangeInches() > 3) // TODO change value
	{
		RightIntakeMotor.Set(-1);
		LeftIntakeMotor.Set(1);
	}
	else
	{
		// Use the A button to eject
		if(OperatorController.GetAButton())
		{
			RightIntakeMotor.Set(1);
			LeftIntakeMotor.Set(-1);
		}
		else
		{
			RightIntakeMotor.Set(0);
			LeftIntakeMotor.Set(0);
		}
	}
}

void Robot::Climb()
{
	// Use the y button to climb
	if(OperatorController.GetYButton())
	{
		ClimbMotor.Set(-1);
	}
	else
	{
		ClimbMotor.Set(0);
	}
}

void Robot::Linkage()
{
	// Use the left y-axis to do the linkage
	LinkageMotor.Set(OperatorController.GetY(GenericHID::JoystickHand::kLeftHand));
}

void Robot::TeleopInit()
{
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	RightIntakeMotor.Set(0);
	LeftIntakeMotor.Set(0);
}

void Robot::TeleopPeriodic()
{
	Drive();
	Elevator();
	Intake();
	Climb();
	Linkage();
	CurrentTest();
}
