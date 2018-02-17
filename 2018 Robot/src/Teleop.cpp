#include "Robot.h"

void Robot::TeleopInit()
{
	//Disable/reset everything from autonomous
	ResetEncoders();
	AngleSensors.Reset();
	AngleController.Disable();
	MaintainAngleController.Disable();
	DistanceController.Disable();

	SmartDashboard::PutBoolean("Record Path", 0);
	SmartDashboard::PutString("Record Output File", "RobotOutputLog.txt");
}

void Robot::TeleopPeriodic()
{
	if(SmartDashboard::GetBoolean("Record Path", 0))
	{
		if(!outf.is_open())
		{
			outf.open("/home/lvuser/" + SmartDashboard::GetString("Record Output File", "RobotOutputLog.txt"),
					std::ios::trunc);
		}

		LogMotorOutput();
	}
	else
	{
		if(outf.is_open())
		{
			outf.close();
		}
	}

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

	// Encoder testing
	SmartDashboard::PutNumber("Front Left Encoder", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
	SmartDashboard::PutNumber("Front Right Encoder", PulsesToInches(FrontRightMotor.GetSelectedSensorPosition(0)));

	//Negative is used to invert the speed since the controller sends inverted values (make forward <--> backward)
	DriveTrain.ArcadeDrive(-speedVal, turnVal);
}
