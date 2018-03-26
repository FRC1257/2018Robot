#include "Robot.h"

std::string WaitForGameData();

void Robot::AutonomousInit()
{
	StopCurrentProcesses();

	// Get the game data from the FMS
	std::string gameData;
	try
	{
		gameData = WaitForGameData();
	}
	catch(const char* error)
	{
		// If we didn't receive any game data, drive to the baseline
		DriverStation::GetInstance().ReportError(error);

		Wait(SmartDashboard::GetNumber("Auto Delay", 0));
		DriveToBaseline();
		return;
	}

	// Add an optional delay to account for other robot auto paths
	Wait(SmartDashboard::GetNumber("Auto Delay", 0));

	switch(AutoLocationChooser->GetSelected())
	{
		case consts::AutoPosition::LEFT_START:
			switch(AutoObjectiveChooser->GetSelected())
			{
				case consts::AutoObjective::SWITCH:
					if(gameData[0] == 'L')
					{
						SmartDashboard::PutString("Auto Path", "Left: Path to Switch " + gameData[0]);
						SidePath(consts::AutoPosition::LEFT_START, gameData[0], gameData[1]);
					}
					else if(gameData[0] == 'R')
					{
						SmartDashboard::PutString("Auto Path", "Left: Path to Switch" + gameData[0]);
						OppositeSwitch(consts::AutoPosition::LEFT_START);
					}
					else
					{
						SmartDashboard::PutString("Auto Path", "Left: Drive to Baseline");
						DriveToBaseline();
					}
					break;
				case consts::AutoObjective::SCALE:
					if(gameData[1] == 'L')
					{
						SmartDashboard::PutString("Auto Path", "Left: Path to Scale " + gameData[1]);
						SidePath(consts::AutoPosition::LEFT_START, 'N', gameData[1]);
					}
					else if(gameData[1] == 'R')
					{
						SmartDashboard::PutString("Auto Path", "Left: Path to Scale " + gameData[1]);
						OppositeScale(consts::AutoPosition::LEFT_START);
					}
					else
					{
						SmartDashboard::PutString("Auto Path", "Left: Drive to Baseline");
						DriveToBaseline();
					}
					break;
				case consts::AutoObjective::BASELINE:
					SmartDashboard::PutString("Auto Path", "Left: Drive to Baseline");
					DriveToBaseline();
					break;
				case consts::AutoObjective::DEFAULT:
				default:
					SmartDashboard::PutString("Auto Path", "Left: Default Path");
					SidePath(consts::AutoPosition::LEFT_START, gameData[0], gameData[1]);
					break;
			}
			break;

		case consts::AutoPosition::RIGHT_START:
			switch(AutoObjectiveChooser->GetSelected())
			{
				case consts::AutoObjective::SWITCH:
					if(gameData[0] == 'R')
					{
						SmartDashboard::PutString("Auto Path", "Right: Path to Switch " + gameData[0]);
						SidePath(consts::AutoPosition::RIGHT_START, gameData[0], gameData[1]);
					}
					else if(gameData[0] == 'L')
					{
						SmartDashboard::PutString("Auto Path", "Right: Path to Switch " + gameData[0]);
						OppositeSwitch(consts::AutoPosition::RIGHT_START);
					}
					else
					{
						SmartDashboard::PutString("Auto Path", "Right: Drive to Baseline");
						DriveToBaseline();
					}
					break;
				case consts::AutoObjective::SCALE:
					if(gameData[1] == 'R')
					{
						SmartDashboard::PutString("Auto Path", "Right: Path to Scale " + gameData[1]);
						SidePath(consts::AutoPosition::RIGHT_START, 'N', gameData[1]);
					}
					else if(gameData[1] == 'L')
					{
						SmartDashboard::PutString("Auto Path", "Right: Path to Scale ");
						OppositeScale(consts::AutoPosition::RIGHT_START);
					}
					else
					{
						SmartDashboard::PutString("Auto Path", "Right: Drive to Baseline");
						DriveToBaseline();
					}
					break;
				case consts::AutoObjective::BASELINE:
					SmartDashboard::PutString("Auto Path", "Right: Drive to Baseline");
					DriveToBaseline();
					break;
				case consts::AutoObjective::DEFAULT:
				default:
					SmartDashboard::PutString("Auto Path", "Right: Default Path");
					SidePath(consts::AutoPosition::LEFT_START, gameData[0], gameData[1]);
					break;
			}
			break;

		case consts::AutoPosition::MIDDLE_START:
			MiddlePath(gameData[0]);
			break;

		default:
			SmartDashboard::PutString("Auto Path", "Driving to Baseline");
			DriveToBaseline();
			break;
	}
}

void Robot::AutonomousPeriodic()
{
	SmartDashboard::PutNumber("Angle", AngleSensors.GetAngle());
	SmartDashboard::PutNumber("Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
}

std::string WaitForGameData()
{
	std::string gameData;
	Timer gameDataTimer;
	gameDataTimer.Start();

	// Poll for the game data for some timeout period or until we've received the data
	while(gameData.length() == 0 && !gameDataTimer.HasPeriodPassed(consts::GAME_DATA_TIMEOUT_S))
	{
		gameData = DriverStation::GetInstance().GetGameSpecificMessage();
		Wait(0.05);
	}
	gameDataTimer.Stop();

	if(gameData.length() == 0)
	{
		throw "Unable to read game data. Driving to Baseline";
	}
	else
	{
		return gameData;
	}
}

void Robot::DriveToBaseline()
{
	SmartDashboard::PutString("Auto Status", "Crossing Baseline");
	DriveDistance(90);
	SmartDashboard::PutString("Auto Status", "Finished crossing Baseline");
}

//Wait until the PID controller has reached the target and the robot is steady
void WaitUntilPIDSteady(PIDController& pidController, PIDSource& pidSource, double timeout)
{
	Timer PIDTimer;
	PIDTimer.Start();

	do
	{
		Wait(0.01);
	}
	while(pidController.OnTarget() && !PIDTimer.HasPeriodPassed(consts::PID_TIMEOUT_S));

	PIDTimer.Stop();
	pidController.Disable();
}

void Robot::DriveDistance(double distance, double timeout)
{
	SmartDashboard::PutString("Auto Status", "Driving a Distance...");
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

	WaitUntilPIDSteady(DistanceController, DistancePID, timeout);

	SmartDashboard::PutString("Auto Status", "Drive complete");
}

void Robot::TurnAngle(double angle, double timeout)
{
	SmartDashboard::PutString("Auto Status", "Rotating...");
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

	RightIntakeMotor.Set(consts::INTAKE_SPEED_WHILE_TURNING);
	LeftIntakeMotor.Set(-consts::INTAKE_SPEED_WHILE_TURNING);

	WaitUntilPIDSteady(AngleController, AngleSensors, timeout);

	SmartDashboard::PutString("Auto Status", "Rotation complete");;
}

void Robot::DriveFor(double seconds, double speed = 0.5)
{
	DriveTrain.ArcadeDrive(speed, 0);
	Wait(seconds);
	DriveTrain.ArcadeDrive(0, 0);
}

//Drives forward a distance, places a power cube, and then backs up
void Robot::DropCube(consts::ElevatorIncrement elevatorSetpoint)
{
	SmartDashboard::PutString("Auto Status", "Dropping Cube...");

	RaiseElevator(elevatorSetpoint);
	EjectCube();
	RaiseElevator(consts::ElevatorIncrement::GROUND);

	SmartDashboard::PutString("Auto Status", "Cube Dropped");
}

void Robot::EjectCube()
{
	SmartDashboard::PutString("Auto Status", "Ejecting Cube...");
	RightIntakeMotor.Set(1);
	LeftIntakeMotor.Set(-1);
	Wait(0.4);
	RightIntakeMotor.Set(0);
	LeftIntakeMotor.Set(0);
	SmartDashboard::PutString("Auto Status", "Ejected Cube");
}

void Robot::RaiseElevator(consts::ElevatorIncrement elevatorSetpoint, double timeout)
{
	double elevatorHeight = consts::ELEVATOR_SETPOINTS[elevatorSetpoint];

	if(elevatorHeight != 0)
	{
		SmartDashboard::PutString("Auto Status", "Raising Elevator...");
		ElevatorMotor.Set(0);
		ElevatorPIDController.SetSetpoint(elevatorHeight);
		ElevatorPIDController.Enable();

		WaitUntilPIDSteady(ElevatorPIDController, ElevatorPID, timeout);
		SmartDashboard::PutString("Auto Status", "Elevator Raised");
	}
}

//Puts the power cube in either the same side scale or same switch switch
void Robot::SidePath(consts::AutoPosition start, char switchPosition, char scalePosition)
{
	SmartDashboard::PutString("Auto Status", "Starting SidePath...");
	//90 for left, -90 for right
	double angle = (start == consts::AutoPosition::LEFT_START) ? 90 : -90;
	//L for left, R for right
	char startPosition = (start == consts::AutoPosition::LEFT_START) ? 'L' : 'R';

	//Cross the baseline
	DriveDistance(148);

	//Check if the switch is nearby, and if it is, place a cube in it
	if(switchPosition == startPosition)
	{
		TurnAngle(angle);
		DriveDistance(22, 2.75);
		DropCube(consts::ElevatorIncrement::GROUND);
		SmartDashboard::PutString("Auto Status", "Finished SidePath");
		return; //End auto just in case the cube misses
	}

	//Otherwise, go forward to a better position
	DriveDistance(100);

	//Check if the scale is nearby, and if it is, place a cube in it
	if(scalePosition == startPosition)
	{
		TurnAngle(-angle);
		DriveDistance(14);

		TurnAngle(angle);
		DriveDistance(56);

		TurnAngle(angle);
		DropCube(consts::ElevatorIncrement::SCALE_HIGH);
		SmartDashboard::PutString("Auto Status", "Finished SidePath");
		return; //End auto just in case the cube misses
	}
}

//Puts a power cube in the switch on the side opposite of the robot
void Robot::OppositeSwitch(consts::AutoPosition start)
{
	SmartDashboard::PutString("Auto Status", "Starting OppositeSwitch...");
	//90 for left, -90 for right
	double angle = (start == consts::AutoPosition::LEFT_START) ? 90 : -90;

	if(SwitchApproachChooser->GetSelected() == consts::SwitchApproach::FRONT)
	{
		DriveDistance(42.5);
		TurnAngle(angle);

		DriveDistance(155);
		TurnAngle(-angle);

		TurnAngle(angle);
		DropCube(consts::ElevatorIncrement::GROUND);
	}
	else
	{
		DriveDistance(211);
		TurnAngle(angle);

		DriveDistance(200);
		TurnAngle(angle);

		DriveDistance(62.5);
		TurnAngle(angle);

		DropCube(consts::ElevatorIncrement::GROUND);
	}
	SmartDashboard::PutString("Auto Status", "Finished OppositeSwitch");
}

//Puts a power cube in the scale on the side opposite of the robot
void Robot::OppositeScale(consts::AutoPosition start)
{
	SmartDashboard::PutString("Auto Status", "Starting OppositeScale...");
	//90 for left, -90 for right
	double angle = (start == consts::AutoPosition::LEFT_START) ? 90 : -90;

	DriveDistance(211);
	TurnAngle(angle);

	DriveDistance(200);
	TurnAngle(-angle);

	DriveDistance(37.5);
	TurnAngle(angle);

	DriveDistance(14);
	TurnAngle(-angle);

	DriveDistance(56);

	DropCube(consts::ElevatorIncrement::SCALE_HIGH);
	SmartDashboard::PutString("Auto Status", "Finished OppositeScale");
}

//Puts a power cube in the switch from the middle position
void Robot::MiddlePath(char switchPosition)
{
	SmartDashboard::PutString("Auto Status", "Starting MiddlePath...");
	double angle = 90;

	//Go forward
	DriveDistance(42.5);

	//Check which way the cube should be placed
	if(SwitchApproachChooser->GetSelected() == consts::SwitchApproach::FRONT)
	{
		//If the cube is being placed from the front
		if(switchPosition == 'L')
		{
			TurnAngle(-angle);
			DriveDistance(80);

			TurnAngle(angle);
			DropCube(consts::ElevatorIncrement::GROUND);
		}
		else if(switchPosition == 'R')
		{
			TurnAngle(angle);
			DriveDistance(29);

			TurnAngle(-angle);
			DriveDistance(106);

			TurnAngle(angle);
			DropCube(consts::ElevatorIncrement::GROUND);
		}
	}
	else
	{
		//If the cube is being placed from the side
		if(switchPosition == 'L')
		{
			TurnAngle(-angle);
			DriveDistance(126);

			TurnAngle(angle);
			DriveDistance(106);

			TurnAngle(angle);
			DriveDistance(60, 2.75);
			DropCube(consts::ElevatorIncrement::GROUND);
		}
		else if(switchPosition == 'R')
		{
			TurnAngle(angle);
			DriveDistance(74);

			TurnAngle(-angle);
			DriveDistance(60, 2.75);
			DropCube(consts::ElevatorIncrement::GROUND);
		}
	}
	SmartDashboard::PutString("Auto Status", "Finished MiddlePath");
}
