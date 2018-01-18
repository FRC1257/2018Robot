#include "Robot.h"

void Robot::TeleopInit()
{
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	IntakeMotor.Set(0);

}

void Robot::TeleopPeriodic()
{
	double speedVal = 0;
	double turnVal = 0;
	int m_desiredStep = 0;
	bool m_elevatorPIDEnabled = false;
	double m_stepVals[5] = {0, 4, 8, 12, 16};

	//Driver Controls

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


	// Operator Controls

	//using left back trigger to lower elevator
	if(inDeadZone(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand)))
	{
		ElevatorMotor.Set(-OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand));
	}
	else
	{
		ElevatorMotor.Set(0);
	}

	//using right back trigger to raise elevator
	if(inDeadZone(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand)))
	{
		ElevatorMotor.Set(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand));
	}
	else
	{
		ElevatorMotor.Set(0);
	}



	if (OperatorController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		m_desiredStep++;
		elevatorPID.SetSetpoint(m_stepVals[m_desiredStep]);
	}
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		elevatorPID.SetSetpoint(0);
	}


	/* prototype preset code
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		if(0 < elevatorEncoder.GetDistance() && elevatorEncoder.GetDistance() < 4) // TODO fix values
		{
			elevatorPID.SetSetpoint(0);
		}
		if(4 < elevatorEncoder.GetDistance() && elevatorEncoder.GetDistance() < 8) // TODO fix values
		{
			elevatorPID.SetSetpoint(4);
		}
		if(8 < elevatorEncoder.GetDistance() && elevatorEncoder.GetDistance() < 12) // TODO fix values
				{
					elevatorPID.SetSetpoint(8);
				}
		if(12 < elevatorEncoder.GetDistance() && elevatorEncoder.GetDistance() < 16) // TODO fix values
				{
					elevatorPID.SetSetpoint(0);
				}
		if(16 < elevatorEncoder.GetDistance()) // TODO fix values
				{
					elevatorPID.SetSetpoint(16);
				}
	}

	if (OperatorController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		if(0 < elevatorEncoder.GetDistance() && elevatorEncoder.GetDistance() < 4) // TODO fix values
		{
			elevatorPID.SetSetpoint(4);
		}
		if(4 < elevatorEncoder.GetDistance() && elevatorEncoder.GetDistance() < 8) // TODO fix values
		{
			elevatorPID.SetSetpoint(8);
		}
		if(8 < elevatorEncoder.GetDistance() && elevatorEncoder.GetDistance() < 12) // TODO fix values
		{
			elevatorPID.SetSetpoint(12);
		}
		if(12 < elevatorEncoder.GetDistance() && elevatorEncoder.GetDistance() < 16) // TODO fix values
		{
			elevatorPID.SetSetpoint(16);
		}
	}
	*/

	//elevatorEncoder.SetDistancePerPulse(1); // come back to check value
	// elevatorEncoder.GetDistance();

	//using b button to intake
	if(OperatorController.GetBButton())
	{
		IntakeMotor.Set(1);
	}
	else
	{
		IntakeMotor.Set(0);
	}

	// using a button to eject
	if(OperatorController.GetAButton())
	{
		IntakeMotor.Set(-1);
	}
	else
	{
		IntakeMotor.Set(0);
	}

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
