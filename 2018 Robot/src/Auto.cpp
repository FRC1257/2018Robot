#include "Robot.h"

void Robot::AutonomousInit()
{
	//Zeroing the angle sensor and encoders
	ResetEncoders();
	AngleSensors.Reset();


	std::string gameData;
	Timer gameDataTimer;
	gameDataTimer.Start();

	// Poll for the game data for some timout period or until we've received the data
	while (gameData.length() == 0 && !gameDataTimer.HasPeriodPassed(consts::GAME_DATA_TIMOUT_S))
	{
		gameData = DriverStation::GetInstance().GetGameSpecificMessage();
	}
	gameDataTimer.Stop();

	// Add an optional delay to account for other robot auto paths
	double delayTime = SmartDashboard::GetNumber("Auto Delay", 0);
	Wait(delayTime);

	// If we didn't receive any game data, drive to the baseline
	if(gameData.length() == 0)
	{
		DriverStation::GetInstance().ReportError("Unable to read game data. Driving to Baseline");
		DriveToBaseline();
		return;
	}

	switch(AutoLocationChooser->GetSelected())
	{
		case consts::AutoPosition::LEFT_START:
			switch(AutoObjectiveChooser->GetSelected())
			{
				case consts::AutoObjective::DEFAULT:
					SidePath(consts::AutoPosition::LEFT_START, gameData[0], gameData[1]);
					break;
				case consts::AutoObjective::SWITCH:
					break;
				case consts::AutoObjective::SCALE:
					break;
				case consts::AutoObjective::BASELINE:
					break;
				default:
					SidePath(consts::AutoPosition::LEFT_START, gameData[0], gameData[1]);
					break;
			}
			break;

		case consts::AutoPosition::RIGHT_START:
			switch(AutoObjectiveChooser->GetSelected())
			{
				case consts::AutoObjective::DEFAULT:
					SidePath(consts::AutoPosition::RIGHT_START, gameData[0], gameData[1]);
					break;
				case consts::AutoObjective::SWITCH:
					break;
				case consts::AutoObjective::SCALE:
					break;
				case consts::AutoObjective::BASELINE:
					break;
				default:
					SidePath(consts::AutoPosition::LEFT_START, gameData[0], gameData[1]);
					break;
			}
			break;

		case consts::AutoPosition::MIDDLE_START:
			MiddlePath(gameData[1]);
			break;

		default:
			DriveToBaseline();
			break;
	}
}

void Robot::AutonomousPeriodic()
{
	SmartDashboard::PutNumber("Angle", AngleSensors.GetAngle());
	SmartDashboard::PutNumber("Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
}

void Robot::DriveToBaseline()
{
	DriveForward(85);
}

//Wait until the PID controller has reached the target and the robot is steady
void WaitUntilPIDSteady(PIDController& pidController, PIDSource& pidSource)
{
	double distance, diff, velocity;
	do
	{
		//Calculate velocity of the PID
		distance = pidSource.PIDGet();
		Wait(0.01);
		diff = pidSource.PIDGet() - distance;
		velocity = diff / 0.01;
	}
	while(!pidController.OnTarget() || abs(velocity) > 0.1);
	pidController.Disable();
}

void Robot::DriveForward(double distance)
{
	//Disable other controllers
	AngleController.Disable();

	//Zeroing the angle sensor and encoders
	ResetEncoders();
	AngleSensors.Reset();

	//Disable test dist output for angle
	AnglePIDOut.SetTestDistOutput(0);
	//Make sure the PID objects know about each other to avoid conflicts
	DistancePID.SetAnglePID(&AnglePIDOut);
	AnglePIDOut.SetDistancePID(&DistancePID);

	//Configure the PID controller to make sure the robot drives straight with the NavX
	MaintainAngleController.Reset();
	MaintainAngleController.SetSetpoint(0);

	//Configure the robot to drive a given distance
	DistanceController.Reset();
	DistanceController.SetSetpoint(distance);

	MaintainAngleController.Enable();
	DistanceController.Enable();

	SmartDashboard::PutNumber("Target Distance", distance);

	WaitUntilPIDSteady(DistanceController, DistancePID);
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
	AngleController.Enable();

	SmartDashboard::PutNumber("Target Angle", angle);

	WaitUntilPIDSteady(AngleController, AngleSensors);
}

void Robot::DriveFor(double seconds, double speed = 0.5)
{
	DriveTrain.ArcadeDrive(speed, 0);
	Wait(seconds);
	DriveTrain.ArcadeDrive(0, 0);
}

void Robot::EjectCube()
{
	RightIntakeMotor.Set(1);
	LeftIntakeMotor.Set(-1);
	Wait(0.4);
	RightIntakeMotor.Set(0);
	LeftIntakeMotor.Set(0);
}

void Robot::RaiseElevator(double distance)
{
	ElevatorMotor.Set(0);
	ElevatorPIDController.SetSetpoint(distance);
	ElevatorPIDController.Enable();

	WaitUntilPIDSteady(ElevatorPIDController, ElevatorPID);
}

void Robot::SidePath(consts::AutoPosition start, char switchPosition, char scalePosition)
{
	//90 for left, -90 for right
	double angle = (start == consts::AutoPosition::LEFT_START) ? 90 : -90;

	//L for left, R for right
	char startPosition = (start == consts::AutoPosition::LEFT_START) ? 'L' : 'R';

	//Cross the baseline
	DriveForward(148);

	//Check if the switch is nearby, and if it is, place a cube in it
	if(switchPosition == startPosition)
	{
		DropCube(switchPosition, 5, false);

		return; //End auto just in case the cube misses
	}

	//Otherwise, go forward to a better position
	DriveForward(100);

	//Check if the scale is nearby, and if it is, place a cube in it
	if(scalePosition == startPosition)
	{
		TurnAngle(-angle);
		DriveForward(14);

		TurnAngle(angle);
		DriveForward(56);

		DropCube(scalePosition, 5, true);

		return; //End auto just in case the cube misses
	}
}

void Robot::OppositeSwitch(consts::AutoPosition start, char switchPosition)
{
	//90 for left, -90 for right
	int multiplier = (start == consts::AutoPosition::LEFT_START) ? 1 : -1;
	double angle = 90 * multiplier;

	DriveForward(42.5);
	TurnAngle(angle);

	DriveForward(155);
	TurnAngle(-angle);

	DropCube(switchPosition, 59, false);
}

void Robot::OppositeScale(consts::AutoPosition start, char scalePosition)
{
	//90 for left, -90 for right
	double angle = (start == consts::AutoPosition::LEFT_START) ? 90 : -90;

	DriveForward(211);
	TurnAngle(angle);

	DriveForward(200);
	TurnAngle(-angle);

	DriveForward(37.5);
	TurnAngle(angle);

	DriveForward(14);
	TurnAngle(-angle);

	DriveForward(56);

	DropCube(scalePosition, 5, true);
}

void Robot::MiddlePath(char switchPosition)
{
	double angle = 90;

	//Go forward
	DriveForward(42.5);

	//Check which way the cube should be placed
	if(MiddleApproachChooser->GetSelected() == consts::MiddleApproach::FRONT)
	{
		//If the cube is being placed from the front
		if(switchPosition == 'L')
		{
			TurnAngle(-angle);
			DriveForward(80);

			DropCube(switchPosition, 78, false);
		}
		else if(switchPosition == 'R')
		{
			TurnAngle(angle);
			DriveForward(29);

			DropCube(switchPosition, 78, false);
		}
	}
	else
	{
		//If the cube is being placed from the side
		if(switchPosition == 'L')
		{
			TurnAngle(-angle);
			DriveForward(126);

			TurnAngle(angle);
			DriveForward(106);

			DropCube(switchPosition, 5, false);
		}
		else if(switchPosition == 'R')
		{
			TurnAngle(angle);
			DriveForward(74);

			TurnAngle(-angle);
			DriveForward(106);

			DropCube(switchPosition, 5, false);
		}
	}
}

void Robot::DropCube(char switchPosition, double driveDistance, bool elevate)
{
	double angle = switchPosition == 'L' ? 90 : -90; //90 for L, -90 for R

	TurnAngle(angle);
	DriveForward(driveDistance);

	//Drop Cube w/ elevate if needed

	DriveForward(-driveDistance);
}
