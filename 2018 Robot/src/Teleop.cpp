#include "Robot.h"

void Robot::TeleopInit()
{
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	RightIntakeMotor.Set(0);
	LeftIntakeMotor.Set(0);
}

void Robot::TeleopPeriodic()
{
	double speedVal = 0;
	double turnVal = 0;
	bool isLowering = false;
	bool linkageDeployed = false; // deployed means outside of robot

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

	int correctedRight = 0;
	int correctedLeft = 0;

	//using left back trigger to lower elevator
	if(inDeadZone(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand)))
	{
		correctedLeft = 0;
	}
	else
	{
		correctedLeft = -OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kLeftHand);
	}

	//using right back trigger to raise elevator
	if(inDeadZone(OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand)))
	{
		correctedRight = 0;
	}
	else
	{
		correctedRight = OperatorController.GetTriggerAxis(GenericHID::JoystickHand::kRightHand);
	}

	ElevatorMotor.Set(correctedRight-correctedLeft);

	//using right bumper to raise to presets and stop
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		if (isLowering)
		{
			ElevatorMotor.Set(0);
			isLowering = false;
		}
		else
		{
		elevatorPID.SetSetpoint(getStepNumber(elevatorEncoder.GetDistance()));
		}
	}
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		isLowering = true;
		elevatorPID.SetSetpoint(getStepNumber(elevatorEncoder.GetDistance()));
	}

	//using b button to intake
	if(OperatorController.GetBButton())
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

	// using y button to climb
	if(OperatorController.GetYButton())
	{
		ClimbMotor.Set(-1);
	}
	else
	{
		ClimbMotor.Set(0);
	}

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
