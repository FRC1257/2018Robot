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
			break;
	}
}

void Robot::AutonomousPeriodic()
{
	SmartDashboard::PutNumber("Angle", AngleSensors.GetAngle());
	SmartDashboard::PutNumber("Distance", PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)));
}

void Robot::LogMotorOutput()
{
	//Record motor output and store it in text file

	//Check if the file can be written to
	if (!outf)
	{
		std::cerr << "MotorOutputLog could not be opened" << std::endl;
		return;
	}

	//Write all motor data
	std::string MotorVoltage = "";

	double leftMotorOutput = deadband(LeftMotors.Get());
	MotorVoltage += std::to_string(leftMotorOutput);
	MotorVoltage += ",";

	double rightMotorOutput = deadband(RightMotors.Get());
	MotorVoltage += std::to_string(rightMotorOutput);
	MotorVoltage += ",";

	double leftIntakeOutput = deadband(LeftIntakeMotor.Get());
	MotorVoltage += std::to_string(leftIntakeOutput);
	MotorVoltage += ",";

	double rightIntakeOutput = deadband(RightIntakeMotor.Get());
	MotorVoltage += std::to_string(rightIntakeOutput);
	MotorVoltage += ",";

	double elevatorOutput = deadband(ElevatorMotor.Get());
	MotorVoltage += std::to_string(elevatorOutput);

	outf << MotorVoltage << std::endl;
}

void Robot::ReadLog()
{
	if(!inf)
	{
		std::cout << "MotorOutputLog could not be read" << std::endl;
		return;
	}

	std::string motorInput;
	std::getline(inf, motorInput);
	std::stringstream streamOfCommands(motorInput);

	std::string motorOutput;

	std::getline(streamOfCommands, motorOutput, ',');
	double lMotor = std::atof(motorOutput.c_str());

	std::getline(streamOfCommands, motorOutput, ',');
	double rMotor = std::atof(motorOutput.c_str());

	std::getline(streamOfCommands, motorOutput, ',');
	double lIntake = std::atof(motorOutput.c_str());

	std::getline(streamOfCommands, motorOutput, ',');
	double rIntake = std::atof(motorOutput.c_str());

	std::getline(streamOfCommands, motorOutput, ',');
	double elev = std::atof(motorOutput.c_str());

	LeftMotors.Set(deadband(lMotor));
	RightMotors.Set(deadband(rMotor));
	LeftIntakeMotor.Set(deadband(lIntake));
	RightIntakeMotor.Set(deadband(rIntake));
	ElevatorMotor.Set(deadband(elev));
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

void Robot::SidePath(consts::AutoPosition start, char switchPosition, char scalePosition)
{
	//90 for left, -90 for right
	int multiplier = start == consts::AutoPosition::LEFT_START ? 1 : -1;
	double angle = 90 * multiplier;

	//L for left, R for right
	char startPosition = start == consts::AutoPosition::LEFT_START ? 'L' : 'R';

	DriveForward(148);

	if(switchPosition == startPosition)
	{
		TurnAngle(angle);
		DriveForward(5);

		//Dump Cube
		DriveForward(-5);

		return; //End auto just in case the cube misses
	}

	DriveForward(100);

	if(scalePosition == startPosition)
	{
		TurnAngle(-angle);
		DriveForward(14);

		TurnAngle(angle);
		DriveForward(56);

		TurnAngle(angle);
		DriveForward(5);

		//Dump Cube
		DriveForward(-5);

		return; //End auto just in case the cube misses
	}
}

void Robot::MiddlePath(char switchPosition)
{
	double angle = 90;

	DriveForward(42.5);

	if(MiddleApproachChooser->GetSelected() == consts::MiddleApproach::FRONT)
	{
		if(switchPosition == 'L')
		{
			TurnAngle(-angle);
			DriveForward(80);

			TurnAngle(angle);
			DriveForward(78);

			//Drop Cube

			DriveForward(-78);
		}
		else if(switchPosition == 'R')
		{
			TurnAngle(angle);
			DriveForward(29);

			TurnAngle(-angle);
			DriveForward(78);

			//Drop Cube

			DriveForward(-78);
		}
	}
	else
	{
		if(switchPosition == 'L')
		{
			//Measurements Later
		}
		else if(switchPosition == 'R')
		{
			//Measurements Later
		}
	}
}
