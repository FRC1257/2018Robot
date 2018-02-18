#include "Robot.h"

void Robot::DisabledInit()
{
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
}

void Robot::DisabledPeriodic()
{

}
