#include "Robot.h"

void Robot::AutonomousInit()
{
	//Zeroing the angle sensor and encoders
	ResetEncoders();
	AngleSensors.Reset();

	std::string gameData;
	DriverStation::GetInstance().WaitForData();
//	while (gameData == NULL) Not sure if this works
//	{
	gameData = DriverStation::GetInstance().GetGameSpecificMessage();
//	}

	double delayTime = SmartDashboard::GetNumber("Auto Delay", 0);
	Wait(delayTime);

	switch(AutoLocationChooser->GetSelected())
	{
		case consts::AutoPosition::LEFT_START:
			SidePath(consts::AutoPosition::LEFT_START, gameData[0], gameData[1]);
			break;

		case consts::AutoPosition::RIGHT_START:
			SidePath(consts::AutoPosition::RIGHT_START, gameData[0], gameData[1]);
			break;

		case consts::AutoPosition::MIDDLE_START:
			MiddlePath(gameData[1]);
			break;

		default:
			//If the robot receives no input, cross the baseline
			DriveForward(85);
			break;
	}
}

void Robot::AutonomousPeriodic()
{
	SmartDashboard::PutNumber("Angle", AngleSensors.GetAngle());
	SmartDashboard::PutNumber("Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
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

	//Wait until the PID controller has reached the target and the robot is steady
	double traveled, distDiff, velocity;
	do
	{
		//Calculate Velocity of Robot
		traveled = PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0));
		Wait(0.01);
		distDiff = PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)) - traveled;
		velocity = distDiff / 0.01;
	}
	while(!DistanceController.OnTarget() || velocity > 0.1);
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
	AngleController.Enable();

	SmartDashboard::PutNumber("Target Angle", angle);

	//Wait until the PID controller has reached the target and the robot is steady
	double turned, angleDiff, angleVelocity;
	do
	{
		//Calculate Angular Velocity of Robot
		turned = AngleSensors.GetAngle();
		Wait(0.01);
		angleDiff = AngleSensors.GetAngle() - turned;
		angleVelocity = angleDiff / 0.01;
	}
	while(!AngleController.OnTarget() || angleVelocity > 0.1);
	AngleController.Disable();
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

	//Wait until the PID controller has reached the target and the robot is steady
	double elevatorDist, elevatorDistDiff, elevatorVelocity;
	do
	{
		//Calculate Elevator Velocity
		elevatorDist = ElevatorEncoder.Get();
		Wait(0.01);
		elevatorDistDiff = ElevatorEncoder.Get() - elevatorDist;
		elevatorVelocity = elevatorDistDiff / 0.01;
	}
	while(!ElevatorPIDController.OnTarget() || elevatorVelocity > 0.1);
	ElevatorPIDController.Disable();
}

void Robot::SidePath(consts::AutoPosition start, char switchPosition, char scalePosition)
{
	//90 for left, -90 for right
	int multiplier = start == consts::AutoPosition::LEFT_START ? 1 : -1;
	double angle = 90 * multiplier;

	//L for left, R for right
	char startPosition = start == consts::AutoPosition::LEFT_START ? 'L' : 'R';

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
