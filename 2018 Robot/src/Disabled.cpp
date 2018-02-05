#include "Robot.h"

void Robot::DisabledInit()
{
	AutoLocationChooser->AddDefault("Left Start", consts::AutoPosition::LEFT_START);
	AutoLocationChooser->AddObject("Middle Start", consts::AutoPosition::MIDDLE_START);
	AutoLocationChooser->AddObject("Right Start", consts::AutoPosition::RIGHT_START);

	AutoObjectiveChooser->AddDefault("Switch", consts::AutoObjective::SWITCH);
	AutoObjectiveChooser->AddObject("Scale", consts::AutoObjective::SCALE);
	AutoObjectiveChooser->AddObject("Baseline", consts::AutoObjective::BASELINE);

	SmartDashboard::PutData("Auto Position", AutoLocationChooser);
	SmartDashboard::PutData("Auto Objective", AutoObjectiveChooser);
	SmartDashboard::PutNumber("Auto Delay", 0);
}

void Robot::DisabledPeriodic()
{

}
