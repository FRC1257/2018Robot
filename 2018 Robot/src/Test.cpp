#include "Robot.h"

void Robot::TestInit()
{

}

void Robot::TestPeriodic()
{
	DriveTest();
//	ElevatorTest();
//	LinkageTest();
//	IntakeTest();
//	ClimbTest();
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

	double leftMotorOutput;
	double rightMotorOutput;
	bool squaredInput = true;

	forwardSpeed = Limit(forwardSpeed);
	turnSpeed = Limit(turnSpeed);

	if(squaredInput)
	{
		forwardSpeed = copysign(forwardSpeed * forwardSpeed, forwardSpeed);
		turnSpeed = copysign(turnSpeed * turnSpeed, turnSpeed);
	}

	double maxInput = copysign(max(abs(forwardSpeed), abs(turnSpeed)), forwardSpeed);

	if(forwardSpeed >= 0.0)
	{
		// First quadrant, else second quadrant
		if(turnSpeed >= 0.0)
		{
		  leftMotorOutput = maxInput;
		  rightMotorOutput = forwardSpeed - turnSpeed;
		}
		else
		{
		  leftMotorOutput = forwardSpeed + turnSpeed;
		  rightMotorOutput = maxInput;
		}
	}
	else
	{
		// Third quadrant, else fourth quadrant
		if(turnSpeed >= 0.0)
		{
		  leftMotorOutput = forwardSpeed + turnSpeed;
		  rightMotorOutput = maxInput;
		}
		else
		{
		  leftMotorOutput = maxInput;
		  rightMotorOutput = forwardSpeed - turnSpeed;
		}
	}

	SmartDashboard::PutNumber("Right Motor Output", Limit(rightMotorOutput));
	SmartDashboard::PutNumber("Left Motor Output", Limit(leftMotorOutput));

	// Set the target velocity for the talons with encoders
	FrontRightMotor.Set(ControlMode::Velocity, Limit(rightMotorOutput) * consts::MAX_VELOCITY);
	FrontLeftMotor.Set(ControlMode::Velocity, Limit(leftMotorOutput) * consts::MAX_VELOCITY);
	// Set the other two motor controllers to follow the talons with encoders
	BackRightMotor.Set(ControlMode::Follower, 4);
	BackLeftMotor.Set(ControlMode::Follower, 1);

	SmartDashboard::PutNumber("Front Right Velocity", FrontRightMotor.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("Front Left Velocity", FrontLeftMotor.GetSelectedSensorVelocity(0));
}

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
