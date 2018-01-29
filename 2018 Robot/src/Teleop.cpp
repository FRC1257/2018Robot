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

	if (OperatorController.GetBumper(GenericHID::JoystickHand::kRightHand))
	{
		m_desiredStep++;
		elevatorPID.SetSetpoint(m_stepVals[m_desiredStep]);
	}
	if (OperatorController.GetBumper(GenericHID::JoystickHand::kLeftHand))
	{
		m_desiredStep = 0;
		elevatorPID.SetSetpoint(m_stepVals[m_desiredStep]);
	}

	//using b button to intake
	if(OperatorController.GetBButton())
	{
		IntakeMotor.Set(-1);
	}
	else
	{
		IntakeMotor.Set(0);
	}

	// using a button to eject
	if(OperatorController.GetAButton())
	{
		IntakeMotor.Set(1);
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
