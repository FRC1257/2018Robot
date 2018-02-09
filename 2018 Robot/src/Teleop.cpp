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
	double xSpeed = Limit(xSpeed);  //WHATS LIMIT?!

	double zRotation = Limit(zRotation);

	double m_maxOutput;

	double leftMotorOutput;
	double rightMotorOutput;

	double maxInput =
	  std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

	if (xSpeed >= 0.0) {
	// First quadrant, else second quadrant
	if (zRotation >= 0.0) {
	  leftMotorOutput = maxInput;
	  rightMotorOutput = xSpeed - zRotation;
	} else {
	  leftMotorOutput = xSpeed + zRotation;
	  rightMotorOutput = maxInput;
	}
	} else {
	// Third quadrant, else fourth quadrant
	if (zRotation >= 0.0) {
	  leftMotorOutput = xSpeed + zRotation;
	  rightMotorOutput = maxInput;
	} else {
	  leftMotorOutput = maxInput;
	  rightMotorOutput = xSpeed - zRotation;
	}
	}

	  FrontRightMotor.Set(Limit(leftMotorOutput) * m_maxOutput); //WHATS LIMIT?!
	  FrontLeftMotor.Set(-Limit(rightMotorOutput) * m_maxOutput);

	  //real drive stuff starts here

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
