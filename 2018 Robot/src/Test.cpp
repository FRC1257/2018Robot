#include "Robot.h"

void Robot::TestInit()
{
	SmartDashboard::PutBoolean("Enable Drive", 0);
	SmartDashboard::PutBoolean("Enable Full Elevator", 0);
	SmartDashboard::PutBoolean("Enable Manual Elevator", 0);
	SmartDashboard::PutBoolean("Enable PID Elevator", 0);
	SmartDashboard::PutBoolean("Enable Linkage", 0);
	SmartDashboard::PutBoolean("Enable Intake", 0);
	SmartDashboard::PutBoolean("Enable Climb", 0);
}

void Robot::TestPeriodic()
{
	if(SmartDashboard::GetBoolean("Enable Drive", 0)) DriveTest();
	if(SmartDashboard::GetBoolean("Enable Full Elevator", 0))
	{
		FullElevatorTest();
		SmartDashboard::PutBoolean("Enable Full Elevator", 0);
		SmartDashboard::PutBoolean("Enable Manual Elevator", 0);
	}
	if(SmartDashboard::GetBoolean("Enable Manual Elevator", 0)) ManualElevatorTest();
	if(SmartDashboard::GetBoolean("Enable PID Elevator", 0)) PIDElevatorTest();
	if(SmartDashboard::GetBoolean("Enable Linkage", 0)) LinkageTest();
	if(SmartDashboard::GetBoolean("Enable Intake", 0)) IntakeTest();
	if(SmartDashboard::GetBoolean("Enable Climb", 0)) ClimbTest();
}


void Robot::RunMotorsTestFor(int numberOfSeconds)
{
	DriveTrain.ArcadeDrive(1, 0);
	Wait(numberOfSeconds / 2);
	DriveTrain.ArcadeDrive(-1, 0);
	Wait(numberOfSeconds / 2);
	DriveTrain.ArcadeDrive(0, 0);
}

void Robot::DriveTest()
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

void Robot::ManualElevatorTest()
{
	// Use the right trigger to manually raise the elevator and
	// the left trigger to lower the elevator
	double raiseElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kRightHand));
	double lowerElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kLeftHand));

	SmartDashboard::PutNumber("RaiseElev", raiseElevatorOutput);
	SmartDashboard::PutNumber("LowerElev", lowerElevatorOutput);

	if(raiseElevatorOutput != 0.0 || lowerElevatorOutput != 0.0)
	{
		ElevatorPIDController.Disable();
		double output = dabs(raiseElevatorOutput) - dabs(lowerElevatorOutput);
		if((output < 0 && ElevatorPID.GetHeightInches() < 5.0) ||
				(output > 0 && ElevatorPID.GetHeightInches() > 65.0))
		{
			output = 0;
		}
		ElevatorMotor.Set(output);
		return;
	}
	else if(!ElevatorPIDController.IsEnabled())
	{
		ElevatorMotor.Set(0);
	}
}

void Robot::PIDElevatorTest()
{
	// Use the right trigger to manually raise the elevator and
	// the left trigger to lower the elevator
	double raiseElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kRightHand));
	double lowerElevatorOutput = applyDeadband(OperatorController.GetTriggerAxis(
			GenericHID::JoystickHand::kLeftHand));

	if(raiseElevatorOutput != 0.0 || lowerElevatorOutput != 0.0)
	{
		double output = dabs(raiseElevatorOutput) - dabs(lowerElevatorOutput);
		if((output < 0 && ElevatorPID.GetHeightInches() < 5.0) ||
				(output > 0 && ElevatorPID.GetHeightInches() > 65.0))
		{
			output = 0;
		}
		ElevatorMotor.Set(output);
		return;
	}
	else if(!ElevatorPIDController.IsEnabled())
	{
		ElevatorMotor.Set(0);
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

void Robot::FullElevatorTest()
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

void Robot::LinkageTest()
{
	// Use the left y-axis to do the linkage
	LinkageMotor.Set(OperatorController.GetY(GenericHID::JoystickHand::kLeftHand));
}

void Robot::IntakeTest()
{
	// Use the B button to intake
	if(OperatorController.GetBButton() && IntakeUltrasonic.GetRangeInches() > 3) // TODO change value
	{
		RightIntakeMotor.Set(0.5);
		LeftIntakeMotor.Set(-0.5);
	}
	else
	{
		// Use the A button to eject if the B button is not being held
		if(OperatorController.GetAButton())
		{
			RightIntakeMotor.Set(-0.5);
			LeftIntakeMotor.Set(0.5);
		}
		else
		{
			RightIntakeMotor.Set(0);
			LeftIntakeMotor.Set(0);
		}
	}
}

void Robot::ClimbTest()
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

void Robot::CurrentTest()
{
	SmartDashboard::PutNumber("FL Current",      FrontLeftMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("FR Current",      FrontRightMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("BL Current",      BackLeftMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("BR Current",      BackRightMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("RIntake Current", RightIntakeMotor.GetOutputCurrent());
	SmartDashboard::PutNumber("LIntake Current", LeftIntakeMotor.GetOutputCurrent());
}

