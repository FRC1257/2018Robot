#include "Robot.h"

void ReadDirectory(const std::string& name, std::vector<std::string>& list)
{
    DIR* dir = opendir(name.c_str());
    struct dirent* dp;
    for(int i = 1; (dp = readdir(dir)) != NULL; i++)
    {
		//Add all nonhidden files
		if(dp->d_name[0] != '.')
		{
			list.push_back(dp->d_name);
		}
    }
    closedir(dir);
}

void Robot::DisabledInit()
{
	StopCurrentProcesses();

	// Configure the SendableChoosers for auto
	AutoLocationChooser->AddDefault("Left Start", consts::AutoPosition::LEFT_START);
	AutoLocationChooser->AddObject("Middle Start", consts::AutoPosition::MIDDLE_START);
	AutoLocationChooser->AddObject("Right Start", consts::AutoPosition::RIGHT_START);

	AutoObjectiveChooser->AddDefault("Default", consts::AutoObjective::DEFAULT);
	AutoObjectiveChooser->AddObject("Switch", consts::AutoObjective::SWITCH);
	AutoObjectiveChooser->AddObject("Scale", consts::AutoObjective::SCALE);
	AutoObjectiveChooser->AddObject("Baseline", consts::AutoObjective::BASELINE);

	SwitchApproachChooser->AddDefault("Front", consts::SwitchApproach::FRONT);
	SwitchApproachChooser->AddObject("Side", consts::SwitchApproach::SIDE);

	SmartDashboard::PutData("Auto Position", AutoLocationChooser);
	SmartDashboard::PutData("Auto Objective", AutoObjectiveChooser);
	SmartDashboard::PutData("Middle Approach", SwitchApproachChooser);
	SmartDashboard::PutNumber("Auto Delay", 0);

	std::vector<std::string> fileNames;
	ReadDirectory(consts::AUTO_PATH, fileNames);

	EchoAutoFileNameChooser->AddDefault("None", "");
	for(std::string file : fileNames)
	{
		EchoAutoFileNameChooser->AddObject(file, file);
	}
	SmartDashboard::PutData("Echo Auto File Name", EchoAutoFileNameChooser);
}

void Robot::DisabledPeriodic()
{

}

void Robot::StopCurrentProcesses()
{
	ResetSensors();
	DisablePIDControllers();
	ZeroMotors();
}

void Robot::ResetSensors()
{
	ResetEncoders();
	AngleSensors.Reset();
}

void Robot::ResetEncoders()
{
	FrontLeftMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
	FrontRightMotor.SetSelectedSensorPosition(0, consts::PID_LOOP_ID, consts::TALON_TIMEOUT_MS);
}

void Robot::DisablePIDControllers()
{
	AngleController.Disable();
	MaintainAngleController.Disable();
	DistanceController.Disable();
	ElevatorPIDController.Disable();
}

void Robot::ZeroMotors()
{
	BackRightMotor.Set(0);
	FrontRightMotor.Set(0);
	FrontLeftMotor.Set(0);
	BackLeftMotor.Set(0);
	LinkageMotor.Set(0);
	RightIntakeMotor.Set(0);
	ClimbMotor.Set(0);
	ElevatorMotor.Set(0);
	LeftIntakeMotor.Set(0);
}
