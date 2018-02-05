#include "Robot.h"

double Robot::GetStepNumber(double elevatorHeight)
{
	for(int i = 0; i < 5; i++)
	{
		if(elevatorHeight < consts::STEPVALS[i])
		{
			return i;
		}
	}
	return 4;
}

//Driver Controls
void Robot::Drive()
{
	double speedVal = 0;
	double turnVal = 0;

	// If they press A, use single stick arcade with the left joystick
	if(DriveController.GetAButton())
	{
		speedVal = DriveController.GetY(GenericHID::JoystickHand::kLeftHand);
		turnVal = DriveController.GetX(GenericHID::JoystickHand::kLeftHand);
	}
	// If they press the left bumper, use the left joystick for forward and
	// backward motion and the right joystick for turning
	else if(DriveController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		speedVal = DriveController.GetY(GenericHID::JoystickHand::kLeftHand);
		turnVal = DriveController.GetX(GenericHID::JoystickHand::kRightHand);
	}
	// If they press the right bumper, use the right joystick for forward and
	// backward motion and the left joystick for turning
	else if(DriveController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		speedVal = DriveController.GetY(GenericHID::JoystickHand::kRightHand);
		turnVal = DriveController.GetX(GenericHID::JoystickHand::kLeftHand);
	}

	if(inDeadZone(speedVal))
	{
		speedVal = 0;
	}
	if(inDeadZone(turnVal))
	{
		turnVal = 0;
	}
	//Negative is used to invert the speed (make forward <--> backward)
	DriveTrain.ArcadeDrive(-speedVal, turnVal);


	/* get gamepad axis */
	double leftYstick = m_joy->GetY(GenericHID::JoystickHand::kLeftHand);
	double motorOutput = m_talon->GetMotorOutputPercent();
	bool button1 = m_joy->GetRawButton(1);

	/* prepare line to print */
	m_sb.append("\tout:");
	m_sb.append(std::to_string(motorOutput));
	m_sb.append("\tcur:");
	m_sb.append(std::to_string(m_talon->GetOutputCurrent()));

	/* on button1 press enter closed-loop mode on target position */
	if (button1)
	{
		/* Position mode - button just pressed */
		m_talon->Set(ControlMode::Current, leftYstick * 40); /* 40 Amps in either direction */
	}
	else
	{
		m_talon->Set(ControlMode::PercentOutput, leftYstick);
	}
	/* if Talon is in position closed-loop, print some more info */
	if (m_talon->GetControlMode() == ControlMode::Current)
	{
		/* append more signals to print when in speed mode. */
		m_sb.append("\terrNative:");
		m_sb.append(std::to_string(m_talon->GetClosedLoopError(CCL_PID_LOOP_IDX)));
		m_sb.append("\ttrg:");
		m_sb.append(std::to_string(leftYstick * 40));
	}
	/* print every ten loops, printing too much too fast is generally bad for performance */
	if (++m_loops >= 10)
	{
		m_loops = 0;
		printf("%s\n", m_sb.c_str());
	}
	m_sb.clear();
}

// Operator Controls
void Robot::Elevator()
{
	int correctedRight = 0;
	int correctedLeft = 0;

	//using left back trigger to lower elevator
	if(inDeadZone(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand)))
	{
		correctedLeft = 0;
	}
	else
	{
		m_inAutomatic = false;
		correctedLeft = -OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand);
	}

	//using right back trigger to raise elevator
	if(inDeadZone(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand)))
	{
		correctedRight = 0;
	}
	else
	{
		m_inAutomatic = false;
		correctedRight = OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand);
	}

	ElevatorMotor.Set(correctedRight - correctedLeft);

	//using right bumper to raise to presets and stop
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		//If elevator is lowering and right bumper is pressed, stop elevator where it is
		if (m_isLowering)
		{
			ElevatorMotor.Set(0);
			m_isLowering = false;
		}
		else
		{
			//If right bumper is being pressed for the first time, changes targetStep to the next highest step
			if (!m_inAutomatic)
			{
				m_inAutomatic = true;
				m_targetStep = GetStepNumber(ElevatorEncoder.GetDistance());
			}
			//if right bumper has already been pressed, go to the next step.
			else if (m_targetStep < 4)
			{
				m_targetStep++;
			}
			ElevatorPID.SetSetpoint(consts::STEPVALS[m_targetStep]);
		}
	}
	//if left bumper is pressed move the elevator to the bottom.
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		m_inAutomatic = false;
		m_isLowering = true;
		ElevatorPID.SetSetpoint(0);
	}
}

void Robot::Intake()
{
	//using b button to intake
	if(OperatorController.GetBButton() && IntakeUltrasonic.GetRangeInches() > 3) // TODO change value
	{
		RightIntakeMotor.Set(-1);
		LeftIntakeMotor.Set(1);
	}
	else
	{
		RightIntakeMotor.Set(0);
		LeftIntakeMotor.Set(0);
	}

	// using a button to eject
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

void Robot::Climb()
{
	// using y button to climb
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
	//using left joystick to do the linkage thing
	if(!inDeadZone(dabs(OperatorController.GetY(GenericHID::JoystickHand::kLeftHand))))
	{
		LinkageMotor.Set(OperatorController.GetY(GenericHID::JoystickHand::kLeftHand));
	}
	else
	{
		LinkageMotor.Set(0);
	}
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
}
