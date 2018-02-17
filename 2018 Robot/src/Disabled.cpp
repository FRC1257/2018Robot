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
	AutoLocationChooser->AddDefault("Left Start", consts::AutoPosition::LEFT_START);
	AutoLocationChooser->AddObject("Middle Start", consts::AutoPosition::MIDDLE_START);
	AutoLocationChooser->AddObject("Right Start", consts::AutoPosition::RIGHT_START);

	AutoObjectiveChooser->AddDefault("Switch", consts::AutoObjective::SWITCH);
	AutoObjectiveChooser->AddObject("Scale", consts::AutoObjective::SCALE);
	AutoObjectiveChooser->AddObject("Baseline", consts::AutoObjective::BASELINE);

	MiddleApproachChooser->AddDefault("Front", consts::MiddleApproach::FRONT);
	MiddleApproachChooser->AddDefault("Side", consts::MiddleApproach::SIDE);

	SmartDashboard::PutData("Auto Position", AutoLocationChooser);
	SmartDashboard::PutData("Auto Objective", AutoObjectiveChooser);
	SmartDashboard::PutData("Middle Approach", MiddleApproachChooser);
	SmartDashboard::PutNumber("Auto Delay", 0);

	std::vector<std::string> fileNames;
	ReadDirectory(consts::AUTO_PATH, fileNames);

	for(std::string file : fileNames)
	{
		EchoAutoFileNameChooser->AddObject(file, file);
	}
	SmartDashboard::PutData("Echo Auto File Name", EchoAutoFileNameChooser);
}

void Robot::DisabledPeriodic()
{

}
