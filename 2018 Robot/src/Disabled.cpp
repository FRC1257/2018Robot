#include "Robot.h"

void Robot::DisabledInit()
{
	AutoLocationChooser->AddDefault("Left Start", constants::AutoPosition::LEFT_START);
	AutoLocationChooser->AddObject("Middle Start", constants::AutoPosition::MIDDLE_START);
	AutoLocationChooser->AddObject("Right Start", constants::AutoPosition::RIGHT_START);

	AutoObjectiveChooser->AddDefault("Switch", constants::AutoObjective::SWITCH);
	AutoObjectiveChooser->AddObject("Scale", constants::AutoObjective::SCALE);
	AutoObjectiveChooser->AddObject("Baseline", constants::AutoObjective::BASELINE);
	AutoObjectiveChooser->AddObject("Stay Still", constants::AutoObjective::STAY_STILL);

	SmartDashboard::PutData("Auto Position", AutoLocationChooser);
	SmartDashboard::PutData("Auto Objective", AutoObjectiveChooser);
	SmartDashboard::PutNumber("Auto Delay(s)", 0);
}

void Robot::DisabledPeriodic()
{

}
