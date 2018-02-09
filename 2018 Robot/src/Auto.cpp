#include "Robot.h"

void Robot::AutonomousInit()
{
	//Zeroing the angle sensor and encoders
	AngleSensors.Reset();
	FrontLeftMotor.SetSelectedSensorPosition(0, consts::PIDLoopIdx, consts::timeoutMs);
	FrontRightMotor.SetSelectedSensorPosition(0, consts::PIDLoopIdx, consts::timeoutMs);

	std::string gameData;
	DriverStation::GetInstance().WaitForData();
	gameData = DriverStation::GetInstance().GetGameSpecificMessage();

	double delayTime = SmartDashboard::GetNumber("Auto Delay", 0);
	Wait(delayTime);

	switch(AutoLocationChooser->GetSelected())
	{
		case consts::AutoPosition::LEFT_START:
			if(gameData[0] == 'L')
			{
				DriveToSwitch(consts::AutoPosition::LEFT_START);
				//Drop powercube
			}
			else if(gameData[1] == 'L')
			{
				DriveToScale(consts::AutoPosition::LEFT_START);
				//Drop powercube at scale height
			}
			else
			{
				DriveForward(70);
			}
			break;

		case consts::AutoPosition::RIGHT_START:
			if(gameData[0] == 'R')
			{
				DriveToSwitch(consts::AutoPosition::RIGHT_START);
				//Drop powercube
			}
			else if(gameData[1] == 'R')
			{
				DriveToScale(consts::AutoPosition::RIGHT_START);
				//Drop powercube at scale height
			}
			else
			{
				DriveForward(70);
			}
			break;

		case consts::AutoPosition::MIDDLE_START:
			MiddleToSwitch(gameData[0]);
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
	FrontLeftMotor.SetSelectedSensorPosition(0, consts::PIDLoopIdx, consts::timeoutMs);
	FrontRightMotor.SetSelectedSensorPosition(0, consts::PIDLoopIdx, consts::timeoutMs);
	AngleSensors.Reset();

	//Disable test dist output for angle
	AnglePIDOut.SetTestDistOutput(0);

	//Make sure the PID objects know about each other to avoid conflicts
	DistancePID.SetAnglePID(&AnglePIDOut);
	AnglePIDOut.SetDistancePID(&DistancePID);

	//Configure the PID controller to make sure the robot drives straight with the NavX
	MaintainAngleController.Reset();
	MaintainAngleController.SetSetpoint(0);
	MaintainAngleController.SetAbsoluteTolerance(0.5);
	MaintainAngleController.SetOutputRange(-1.0, 1.0);

	//Configure the robot to drive a given distance
	DistanceController.Reset();
	DistanceController.SetSetpoint(distance);
	DistanceController.SetAbsoluteTolerance(1);
	DistanceController.SetOutputRange(-1.0, 1.0);

	MaintainAngleController.Enable();
	DistanceController.Enable();

	SmartDashboard::PutNumber("Target Distance", distance);

	while(!DistanceController.OnTarget())
	{
		Wait(0.01);
	}
	DistanceController.Disable();
}

void Robot::TurnAngle(double angle)
{
	//Disable other controllers
	DistanceController.Disable();
	MaintainAngleController.Disable();

	//Zeroing the angle sensor
	AngleSensors.Reset();

	//Disable test dist output for angle
	AnglePIDOut.SetTestDistOutput(0);

	//Remove the pointers since only one PID is being used
	DistancePID.SetAnglePID(nullptr);
	AnglePIDOut.SetDistancePID(nullptr);

	AngleController.Reset();
	AngleController.SetSetpoint(angle);
	AngleController.SetAbsoluteTolerance(0.5);
	AngleController.SetOutputRange(-1.0, 1.0);
	AngleController.Enable();

	SmartDashboard::PutNumber("Target Angle", angle);

	while(!AngleController.OnTarget())
	{
		Wait(0.01);
	}
	AngleController.Disable();
}

void Robot::DriveFor(double seconds, double speed = 0.5)
{
	DriveTrain.ArcadeDrive(speed, 0);
	Wait(seconds);
	DriveTrain.ArcadeDrive(0, 0);
}

void Robot::DriveToSwitch(consts::AutoPosition startPosition)
{
	double initialTurnAng = 90;
	if (startPosition == consts::AutoPosition::RIGHT_START)
	{
		// Mirror the turns for the left side by multiplying by -1
		initialTurnAng *= -1;
	}
	DriveForward(130);
	TurnAngle(-initialTurnAng);
	DriveForward(25);
}


void Robot::MiddleToSwitch(char switchPosition)
{
	DriveForward(25);
	double initialTurnAng = 90;
	if (switchPosition == 'L')
	{
		//Mirror the turns for the left side by multiplying by -1
		initialTurnAng = -90;
	}
	TurnAngle(initialTurnAng);
	DriveForward(80);
	TurnAngle(-initialTurnAng);
	DriveForward(78);
	//Drop powercube
	DriveForward(-78);
}

void Robot::DriveToScale(consts::AutoPosition startPosition)
{
	double initialTurnAng = 90;
	if(startPosition == consts::AutoPosition::RIGHT_START)
	{
		//Mirror the turns for the left side by multiplying by -1
		initialTurnAng = -90;
	}
	DriveForward(130);
	DriveForward(70);
	TurnAngle(initialTurnAng);
	DriveForward(30);
	TurnAngle(-initialTurnAng);
	DriveForward(86);
	TurnAngle(-initialTurnAng);
	DriveForward(36);
}
