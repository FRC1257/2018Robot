#include "Robot.h"

std::string WaitForGameData();

/*
 * Fix Measurements for:
 * 	Same Side Switch                  (NEEDS TESTING)
 * 	Same Side Scale                   (NEEDS TESTING) (NEEDS ELEVATOR)
 * 	Opposite Side Switch from Side    (NEEDS TESTING) (NUMBERS MUST BE CHECKED)
 * 	Opposite Side Scale               (NEEDS TESTING) (NUMBERS MUST BE CHECKED) (NEEDS ELEVATOR)
 *
 * 	Opposite Side Switch from Front   (NOT DOING)
 * 	Middle Switch from Side           (NOT DOING)
 */

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
					SidePath(consts::AutoPosition::RIGHT_START, gameData[0], gameData[1]);
					break;
			}
			break;

		case consts::AutoPosition::MIDDLE_START:
			SmartDashboard::PutString("Auto Path", "Middle Switch");
			MiddlePath(gameData[0]);
			break;

		default:
			SmartDashboard::PutString("Auto Path", "Middle Switch");
			MiddlePath(gameData[0]);
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
	DriveDistance(150);
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
	while(!pidController.OnTarget() && !PIDTimer.HasPeriodPassed(timeout));

	PIDTimer.Stop();
	pidController.Disable();
}

void Robot::DriveDistance(double distance, double timeout)
{
	SmartDashboard::PutString("Auto Status", "Driving a Distance...");
	//Disable other controllers
	AngleController.Disable();

	//Zeroing the angle sensor and encoders
	ResetDriveEncoders();
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

//	RightIntakeMotor.Set(consts::INTAKE_SPEED_WHILE_TURNING);
//	LeftIntakeMotor.Set(-consts::INTAKE_SPEED_WHILE_TURNING);

	WaitUntilPIDSteady(AngleController, AngleSensors, timeout);

//	RightIntakeMotor.Set(0);
//	LeftIntakeMotor.Set(0);

	SmartDashboard::PutString("Auto Status", "Rotation complete");;
}

void Robot::DriveFor(double seconds, double speed)
{
	DriveTrain.ArcadeDrive(speed, 0);
	Wait(seconds);
	DriveTrain.ArcadeDrive(0, 0);
}

//Raises elevator, places a power cube, and then lowers elevator
void Robot::DropCube(consts::ElevatorIncrement elevatorSetpoint)
{
	SmartDashboard::PutString("Auto Status", "Dropping Cube...");
//	RaiseElevator(elevatorSetpoint);
	EjectCube();
//	RaiseElevator(consts::ElevatorIncrement::GROUND);
	SmartDashboard::PutString("Auto Status", "Cube Dropped");
}

void Robot::EjectCube(double intakeSpeed)
{
	SmartDashboard::PutString("Auto Status", "Ejecting Cube...");
	RightIntakeMotor.Set(-intakeSpeed);
	LeftIntakeMotor.Set(intakeSpeed);
	Wait(1.0);
	RightIntakeMotor.Set(0);
	LeftIntakeMotor.Set(0);
	SmartDashboard::PutString("Auto Status", "Ejected Cube");
}

void Robot::RaiseElevator(consts::ElevatorIncrement elevatorSetpoint, double timeout)
{
	double elevatorHeight = consts::ELEVATOR_SETPOINTS[elevatorSetpoint];

	// If the difference between heights is significant, lift/lower the elevator
	if(dabs(elevatorHeight - ElevatorPID.PIDGet()) > consts::ELEVATOR_PID_DEADBAND)
	{
		SmartDashboard::PutString("Auto Status", "Raising Elevator...");
		RightElevatorMotor.Set(0);
		ElevatorPIDController.SetSetpoint(elevatorHeight);
		if(elevatorHeight > ElevatorPID.PIDGet())
		{
			ElevatorPIDController.SetPID(consts::ELEVATOR_PID_CONSTANTS_RISING[0],
					consts::ELEVATOR_PID_CONSTANTS_RISING[1],
					consts::ELEVATOR_PID_CONSTANTS_RISING[2]);
		}
		else
		{
			ElevatorPIDController.SetPID(consts::ELEVATOR_PID_CONSTANTS_LOWERING[0],
					consts::ELEVATOR_PID_CONSTANTS_LOWERING[1],
					consts::ELEVATOR_PID_CONSTANTS_LOWERING[2]);
		}
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

	// Get to middle of switch
	DriveDistance(150);

	//Check if the switch is nearby, and if it is, place a cube in it
	if(switchPosition == startPosition)
	{
		TurnAngle(angle);
		DriveDistance(18.25, 2.75);
		DropCube(consts::ElevatorIncrement::GROUND);
		SmartDashboard::PutString("Auto Status", "Finished SidePath");
		return; //End auto just in case the cube misses
	}

	//Otherwise, go forward to a better position
	DriveDistance(103);

	//Check if the scale is nearby, and if it is, place a cube in it
	if(scalePosition == startPosition)
	{
		TurnAngle(angle / 2.0);
		DriveDistance(18.7);

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
	double secondAngle = (start == consts::AutoPosition::LEFT_START) ? 130 : -130;

	if(SwitchApproachChooser->GetSelected() == consts::SwitchApproach::FRONT)
	{
		// This path should never be selected because the
//		DriveDistance(42.5);
//		TurnAngle(angle);
//
//		DriveDistance(155);
//		TurnAngle(-angle);
//
//		TurnAngle(angle);
//		DropCube(consts::ElevatorIncrement::GROUND);
	}
	else
	{
		DriveDistance(215.4);
		TurnAngle(angle);

		DriveDistance(171.5);
		TurnAngle(angle);

		DriveDistance(6.85);

		//TENTATIVE, MIGHT NEED TO RAISE ELEVATOR
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
	double secondAngle = (start == consts::AutoPosition::LEFT_START) ? -120 : 120;

	DriveDistance(215.4);
	TurnAngle(angle);

	DriveDistance(245);
	TurnAngle(secondAngle);

	DriveDistance(55.25);

	DropCube(consts::ElevatorIncrement::SCALE_HIGH);
	SmartDashboard::PutString("Auto Status", "Finished OppositeScale");
}

//Puts a power cube in the switch from the middle position
void Robot::MiddlePath(char switchPosition)
{
	SmartDashboard::PutString("Auto Status", "Starting MiddlePath...");
	double angle = 90;

	//Go forward
	DriveDistance(48);

	//Check which way the cube should be placed
	if(SwitchApproachChooser->GetSelected() == consts::SwitchApproach::SIDE)
	{
		//If the cube is being placed from the side
		if(switchPosition == 'L')
		{
			TurnAngle(-angle);
			DriveDistance(126);

			TurnAngle(angle);
			DriveDistance(106);

			TurnAngle(angle);
			DropCube(consts::ElevatorIncrement::GROUND);
		}
		else if(switchPosition == 'R')
		{
			TurnAngle(angle);
			DriveDistance(74);

			TurnAngle(-angle);
			DriveDistance(106);

			TurnAngle(angle);

			DropCube(consts::ElevatorIncrement::GROUND);
		}
	}
	else
	{
		//If the cube is being placed from the front
		if(switchPosition == 'L')
		{
			TurnAngle(-angle);
			DriveDistance(52);

			TurnAngle(angle);
			DriveDistance(60, 2.75);

			DropCube(consts::ElevatorIncrement::GROUND);
		}
		else if(switchPosition == 'R')
		{
			TurnAngle(angle);
			DriveDistance(52);

			TurnAngle(-angle);
			DriveDistance(60, 2.75);

			DropCube(consts::ElevatorIncrement::GROUND);
		}
	}
	SmartDashboard::PutString("Auto Status", "Finished MiddlePath");
}
