#include "Robot.h"

double Robot::GetClosestStepNumber()
{
	double currentHeight = ElevatorPID.GetHeightInches();
	for(int i = 0; i < consts::NUM_ELEVATOR_SETPOINTS; i++)
	{
		// If the elevator is directly below a given setpoint, go to that setpoint
		if(currentHeight < consts::ELEVATOR_SETPOINTS[i])
		{
			return i;
		}
	}
	return consts::NUM_ELEVATOR_SETPOINTS - 1;
}

bool isApproachingMechanicalStop(double output, double currentHeight)
{
	// If the elevator is moving down towards the bottom
	if(output < 0 && currentHeight < consts::ELEVATOR_SETPOINTS[1])
	{
		return true;
	}
	// If the elevator is moving up towards the top
	else if (output > 0 && currentHeight > consts::ELEVATOR_SETPOINTS[consts::NUM_ELEVATOR_SETPOINTS - 2])
	{
		return true;
	}
	else
	{
		return false;
	}
}

// Prevent the elevator from reaching its hard stops
double Robot::CapElevatorOutput(double output, bool safetyModeEnabled)
{
	double currentHeight = ElevatorPID.GetHeightInches();

	// If we're trying to run the elevator down after reaching the bottom or trying
	// to run it up after reaching the max height, set the motor output to 0
	if((output < 0 && currentHeight < consts::ELEVATOR_SETPOINTS[0]))// ||
			//(output > 0 && currentHeight > consts::ELEVATOR_SETPOINTS[consts::NUM_ELEVATOR_SETPOINTS - 1]))
	{
		output = 0;
	}
	else if(safetyModeEnabled)
	{
		// If the elevator is approaching a mechanical stop, slow down the motor
		if(isApproachingMechanicalStop(output, currentHeight))
		{
			output *= consts::ELEVATOR_SPEED_REDUCTION;
		}
	}

	return output;
}

bool Robot::IsElevatorTooHigh()
{
	return ElevatorPID.PIDGet() > 12.5;
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

	// Ensure the robot doesn't drive at full speed while the elevator is up
	if(IsElevatorTooHigh())
	{
		forwardSpeed *= consts::DRIVE_SPEED_REDUCTION;
		turnSpeed *= consts::DRIVE_SPEED_REDUCTION;
	}

	SmartDashboard::PutBoolean("Drive Speed Reduction?", IsElevatorTooHigh());

	// Negative is used to make forward positive and backwards negative
	// because the y-axes of the XboxController are natively inverted
	DriveTrain.ArcadeDrive(-forwardSpeed, turnSpeed);
}

// Operator controls. This function or the Elevator() function will not be run during
// the same teleopPeriodic() loop
void Robot::ManualElevator()
{
	// Use the right trigger to manually raise the elevator and
	// the left trigger to lower the elevator
	double raiseElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kRightHand));
	double lowerElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kLeftHand));

	if(raiseElevatorOutput != 0.0 || lowerElevatorOutput != 0.0)
	{
		ElevatorPIDController.Disable();
		double output = CapElevatorOutput(dabs(raiseElevatorOutput) - dabs(lowerElevatorOutput));
		ElevatorMotor.Set(output);
		return;
	}
	else
	{
		ElevatorMotor.Set(0);
	}
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

	SmartDashboard::PutNumber("RaiseElev", raiseElevatorOutput);
	SmartDashboard::PutNumber("LowerElev", lowerElevatorOutput);

	// If either triggers are being pressed, disable the PID and
	// set the motor to the given speed
	if(raiseElevatorOutput != 0.0 || lowerElevatorOutput != 0.0)
	{
		ElevatorPIDController.Disable();
		double output = CapElevatorOutput(dabs(raiseElevatorOutput) - dabs(lowerElevatorOutput));
		ElevatorMotor.Set(output);
		return;
	}
	else if(!ElevatorPIDController.IsEnabled())
	{
		ElevatorMotor.Set(0);
	}

	// Automatic Mode is controlled by both bumpers
	if (OperatorController.GetBumperPressed(GenericHID::JoystickHand::kRightHand))
	{
		// If elevator is lowering and the right bumper is pressed, stop elevator where it is
		if (m_isElevatorLowering)
		{
			ElevatorMotor.Set(0);
			m_isElevatorLowering = false;
			ElevatorPIDController.Disable();
		}
		else
		{
			// If right bumper is being pressed for the first time, increase the desired preset by 1
			if (!ElevatorPIDController.IsEnabled())
			{
				m_targetElevatorStep = GetClosestStepNumber();
			}
			// If right bumper has already been pressed, go to the next step.
			else if (m_targetElevatorStep < consts::NUM_ELEVATOR_SETPOINTS - 1)
			{
				m_targetElevatorStep++;
			}
			ElevatorPIDController.SetSetpoint(consts::ELEVATOR_SETPOINTS[m_targetElevatorStep]);
			ElevatorPIDController.Enable();
			m_isElevatorLowering = false;
		}
	}
	// The left bumper will lower the elevator to the bottom
	if (OperatorController.GetBumperPressed(GenericHID::JoystickHand::kLeftHand))
	{
		m_isElevatorLowering = true;
		ElevatorPIDController.SetSetpoint(consts::ELEVATOR_SETPOINTS[0]);
		ElevatorPIDController.Enable();
	}
}

void Robot::Intake()
{
	// Use the Right Y-axis for variable intake speed
	double intakeSpeed = applyDeadband(OperatorController.GetY(GenericHID::kRightHand));
	if(intakeSpeed != 0)
	{
		// If you're spinning the intaking a cube, but it is already within the intake, set the speed to 0
		if(intakeSpeed < 0 && IntakeUltrasonic.GetRangeInches() < consts::MIN_DISTANCE_TO_CUBE)
		{
			intakeSpeed = 0;
		}
		RightIntakeMotor.Set(intakeSpeed);
		LeftIntakeMotor.Set(-intakeSpeed);
	}

	// If the robot isn't using variable intake control, use the B button to intake cubes.
	// The X button overrides the IntakeUltrasonic's safety feature
	else if((OperatorController.GetXButton() && IntakeUltrasonic.GetRangeInches() > consts::MIN_DISTANCE_TO_CUBE) ||
			OperatorController.GetBButton())
	{
		RightIntakeMotor.Set(consts::INTAKE_SPEED);
		LeftIntakeMotor.Set(-consts::INTAKE_SPEED);
	}
	else
	{
		// Use the A button to eject
		if(OperatorController.GetAButton())
		{
			RightIntakeMotor.Set(-consts::INTAKE_SPEED);
			LeftIntakeMotor.Set(consts::INTAKE_SPEED);
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
		ClimbMotor.Set(1);
	}
	else if(OperatorController.GetStartButton())
	{
		ClimbMotor.Set(-1);
	}
	else
	{
		ClimbMotor.Set(0);
	}

	SmartDashboard::PutNumber("Elevator Height", ElevatorPID.PIDGet());
}

void Robot::Linkage()
{
	// Use the left y-axis to do the linkage
	LinkageMotor.Set(OperatorController.GetY(GenericHID::JoystickHand::kLeftHand));
}

void Robot::TeleopInit()
{
	StopCurrentProcesses();
}

void Robot::TeleopPeriodic()
{
	Drive();
	ManualElevator();
	Intake();
	Climb();
	Linkage();
}
