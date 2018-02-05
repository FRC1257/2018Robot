#include "Robot.h"

void Robot::ElevatorTest()
{
	bool inAutomatic = false;
	bool isLowering = false;
	int targetStep = 0;

	int correctedRight = 0;
	int correctedLeft = 0;

	//using left back trigger to lower elevator
	if(inDeadZone(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand)))
	{
		correctedLeft = 0;
	}
	else
	{
		inAutomatic = false;
		correctedLeft = -OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand);
	}

	//using right back trigger to raise elevator
	if(inDeadZone(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand)))
	{
		correctedRight = 0;
	}
	else
	{
		inAutomatic = false;
		correctedRight = OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand);
	}

	ElevatorMotor.Set(0.25*(correctedRight-correctedLeft));

	//using right bumper to raise to presets and stop
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		//If elevator is lowering and right bumper is pressed, stop elevator where it is
		if (isLowering)
		{
			ElevatorMotor.Set(0);
			isLowering = false;
		}
		else
		{
			//If right bumper is being pressed for the first time, changes targetStep to the next highest step
			if (!inAutomatic)
			{
				inAutomatic = true;
				targetStep = GetStepNumber(ElevatorEncoder.GetDistance());
			}
			//if right bumper has already been pressed, go to the next step.
			else if (targetStep < 4)
			{
				targetStep++;
			}
			ElevatorPID.SetSetpoint(consts::STEPVALS[targetStep]);
		}
	}
	//if left bumper is pressed move the elevator to the bottom.
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		inAutomatic = false;
		isLowering = true;
		ElevatorPID.SetSetpoint(0);
	}
}

void Robot::LinkageTest()
{
	//using left joystick to do the linkage thing
	if(!inDeadZone(dabs(OperatorController.GetY(GenericHID::JoystickHand::kLeftHand))))
	{
		LinkageMotor.Set(0.25*OperatorController.GetY(GenericHID::JoystickHand::kLeftHand));
	}
	else
	{
		LinkageMotor.Set(0);
	}
}

void Robot::IntakeTest()
{
	//using b button to intake
	if(OperatorController.GetBButton() && IntakeUltrasonic.GetRangeInches() > 3) // TODO change value
	{
		RightIntakeMotor.Set(0.25*(-1));
		LeftIntakeMotor.Set(0.25*1);
	}
	else
	{
		RightIntakeMotor.Set(0);
		LeftIntakeMotor.Set(0);
	}

	// using a button to eject
	if(OperatorController.GetAButton())
	{
		RightIntakeMotor.Set(0.25*1);
		LeftIntakeMotor.Set(0.25*(-1));
	}
	else
	{
		RightIntakeMotor.Set(0);
		LeftIntakeMotor.Set(0);
	}

}

void Robot::ClimbTest()
{
	// using y button to climb
	if(OperatorController.GetYButton())
	{
		ClimbMotor.Set(0.25*(-1));
	}
	else
	{
		ClimbMotor.Set(0);
	}

}

void Robot::TestInit()
{

}

void Robot::TestPeriodic()
{
	ElevatorTest();
	LinkageTest();
	IntakeTest();
	ClimbTest();
}
