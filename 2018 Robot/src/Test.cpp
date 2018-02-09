#include "Robot.h"

void Robot::TestInit()
{
	SmartDashboard::PutBoolean("Reset Angle", 0);
}

void Robot::TestPeriodic()
{
	SmartDashboard::PutNumber("R Trigger Val", DriveController.GetTriggerAxis(GenericHID::kRightHand));
	SmartDashboard::PutNumber("Auto Pos Val", (int) AutoLocationChooser->GetSelected());
	SmartDashboard::PutNumber("Auto Obj Val", (int) AutoObjectiveChooser->GetSelected());
	if(SmartDashboard::GetBoolean("Reset Angle", 0)) AngleSensors.Reset();
	SmartDashboard::PutNumber("Angle Sensor", AngleSensors.GetAngle());
}

void Robot::MaintainHeadingTest()
{
	//Disable other controllers
	AngleController.Disable();
	DistanceController.Disable();

	//Zeroing the angle sensor
	AngleSensors.Reset();

	//Enable a testing dist output
	AnglePIDOut.SetTestDistOutput(0.35);

	//Remove the pointers since only one PID is being used
	DistancePID.SetAnglePID(nullptr);
	AnglePIDOut.SetDistancePID(nullptr);

	//Configure the PID controller to make sure the robot drives straight with the NavX
	MaintainAngleController.Reset();
	MaintainAngleController.SetSetpoint(0);
	MaintainAngleController.SetAbsoluteTolerance(1);
	MaintainAngleController.SetInputRange(-180.0, 180.0);
	MaintainAngleController.SetContinuous(true);
	MaintainAngleController.SetOutputRange(-1.0, 1.0);

	MaintainAngleController.Enable();
}

void Robot::DriveDistanceTest(double distance)
{
	//Disable other controllers
	AngleController.Disable();

	//Zeroing the angle sensor and encoders
	FrontLeftMotor.SetSelectedSensorPosition(0, consts::PIDLoopIdx, consts::timeoutMs);
	FrontRightMotor.SetSelectedSensorPosition(0, consts::PIDLoopIdx, consts::timeoutMs);
	AngleSensors.Reset();

	//Disable test dist output for angle
	AnglePIDOut.SetTestDistOutput(0);

	//Make sure the PID objects know about each other to avoid conflicts
	DistancePID.SetAnglePID(&AnglePIDOut);
	AnglePIDOut.SetDistancePID(&DistancePID);

	//Configure the PID controller to make sure the robot drives straight with the NavX
	MaintainAngleController.Reset();
	MaintainAngleController.SetSetpoint(0);
	MaintainAngleController.SetAbsoluteTolerance(0.5);
	MaintainAngleController.SetOutputRange(-1.0, 1.0);

	//Configure the robot to drive a given distance
	DistanceController.Reset();
	DistanceController.SetSetpoint(distance);
	DistanceController.SetAbsoluteTolerance(1);
	DistanceController.SetOutputRange(-1.0, 1.0);

	MaintainAngleController.Enable();
	DistanceController.Enable();

	SmartDashboard::PutNumber("Target Distance", distance);
}


void Robot::TurnAngleTest(double angle)
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
	AngleController.SetAbsoluteTolerance(0.5);
	AngleController.SetOutputRange(-1.0, 1.0);
	AngleController.Enable();

	SmartDashboard::PutNumber("Target Angle", angle);
}

