#include "Robot.h"

void Robot::AutonomousInit()
{
	//Zeroing the angle sensor and encoders
	AngleSensors.Reset();
	ResetEncoders();

	std::string gameData;
	DriverStation::GetInstance().WaitForData();
	gameData = DriverStation::GetInstance().GetGameSpecificMessage();

	double delayTime = SmartDashboard::GetNumber("Auto Delay", 0);
	Wait(delayTime);

	switch(AutoLocationChooser->GetSelected())
	{
		case consts::AutoPosition::LEFT_START:
			DriveForward(130);
			if(gameData[0] == 'L')
			{
				TurnAngle(-90);
				DriveForward(25);
			}
			else if(gameData[1] == 'L')
			{
				DriveForward(70);
				TurnAngle(90);
				DriveForward(30);
				TurnAngle(-90);
				DriveForward(86);
				TurnAngle(-90);
				DriveForward(36);
			}
			else
			{
				DriveForward(70);
			}
			break;
		case consts::AutoPosition::MIDDLE_START:
			if(gameData[0] == 'L')
			{

			}
			else if(gameData[0] == 'R')
			{

			}
			break;
		case consts::AutoPosition::RIGHT_START:
			DriveForward(130);
			if(gameData[0] == 'R')
			{
				TurnAngle(90);
				DriveForward(25);
			}
			else if(gameData[1] == 'R')
			{
				DriveForward(70);
				TurnAngle(-90);
				DriveForward(30);
				TurnAngle(90);
				DriveForward(86);
				TurnAngle(90);
				DriveForward(36);
			}
			else
			{
				DriveForward(70);
			}
			break;
		default:

			break;
	}
//	DriveForward(36);
}


void Robot::AutonomousPeriodic()
{
	SmartDashboard::PutNumber("Angle", AngleSensors.GetAngle());
	SmartDashboard::PutNumber("Encoder", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
}

void Robot::DriveForward(double distance)
{
	//Disable other controllers
	AngleController.Disable();

	//Zeroing the angle sensor and encoders
	ResetEncoders();
	AngleSensors.Reset();

	//Make sure the PID objects know about each other to avoid conflicts
	DistancePID.SetAnglePID(&AnglePIDOut);
	AnglePIDOut.SetDistancePID(&DistancePID);

	//Configure the PID controller to make sure the robot drives straight with the NavX
	MaintainAngleController.Reset();
	MaintainAngleController.SetSetpoint(0);
	MaintainAngleController.SetAbsoluteTolerance(1);
	MaintainAngleController.SetInputRange(-180.0, 180.0);
	MaintainAngleController.SetContinuous(true);
	MaintainAngleController.SetOutputRange(-1.0, 1.0);

	//Configure the robot to drive a given distance
	DistanceController.Reset();
	DistanceController.SetSetpoint(distance);
	DistanceController.SetPercentTolerance(1);
	DistanceController.SetOutputRange(-1.0, 1.0);

	MaintainAngleController.Enable();
	DistanceController.Enable();

	SmartDashboard::PutNumber("Target Distance", distance);
}

void Robot::TurnAngle(double angle)
{
	//Disable other controllers
	DistanceController.Disable();
	MaintainAngleController.Disable();

	//Zeroing the angle sensor
	AngleSensors.Reset();

	//Remove the pointers since only one PID is being used
	DistancePID.SetAnglePID(nullptr);
	AnglePIDOut.SetDistancePID(nullptr);

	AngleController.Reset();
	AngleController.SetSetpoint(angle);
	AngleController.SetAbsoluteTolerance(1);
	AngleController.SetInputRange(-180.0, 180.0);
	AngleController.SetContinuous(true);
	AngleController.SetOutputRange(-1.0, 1.0);

	AngleController.Enable();

	SmartDashboard::PutNumber("Target Angle", angle);
}

void Robot::DriveFor(double seconds, double speed = 0.5)
{
	DriveTrain.ArcadeDrive(speed, 0);
	Wait(seconds);
	DriveTrain.ArcadeDrive(0, 0);
}

