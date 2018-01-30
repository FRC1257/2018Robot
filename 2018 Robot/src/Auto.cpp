#include "Robot.h"

void Robot::AutonomousInit()
{
	SmartDashboard::PutString("DB/String 0", "Robot At Left Position");
	SmartDashboard::PutString("DB/String 1", "Robot At Middle Position");
	SmartDashboard::PutString("DB/String 2", "Robot At Right Position");

	std::string gameData;
	DriverStation::GetInstance().WaitForData();

	gameData = DriverStation::GetInstance().GetGameSpecificMessage();

	if(gameData[0] == 'L') // if left side switch
	{
		if(SmartDashboard::GetBoolean("DB/Button 0", false)) // if robot on left side
		{

		}
		else if(SmartDashboard::GetBoolean("DB/Button 1", false)) // if robot on middle side
		{

		}
		else if(SmartDashboard::GetBoolean("DB/Button 2", false)) // if robot on right side
		{

		}
	}
	else if(gameData[0] == 'R')  // if right side switch
	{
		if(SmartDashboard::GetBoolean("DB/Button 0", false)) // if robot on left side
		{

		}
		else if(SmartDashboard::GetBoolean("DB/Button 1", false)) // if robot on middle side
		{

		}
		else if(SmartDashboard::GetBoolean("DB/Button 2", false)) // if robot on right side
		{

		}
	}

	TurnAngle(90);
}

void Robot::AutonomousPeriodic()
{

}

//void Robot::DriveFor(double distance, double speed)
//{
//	double targetDistance = distance + PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0));
//	while(PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)) < targetDistance) //Not sure which motor to test the encoder for - Omay
//	{
//		DriveTrain.ArcadeDrive(speed, 0);
//	}
//
//	DriveTrain.ArcadeDrive(0, 0);
//}

void Robot::DriveForward(double distance)
{
	FrontLeftMotor.SetSelectedSensorPosition(0, 0, 10); //FrontLeft is placeholder until we learn which motor has an encoder
	Gyro.Reset();

	//Make sure the PID objects know about each other to avoid conflicts
	DistancePID.SetAnglePID(&AnglePID);
	AnglePID.SetDistancePID(&DistancePID);

	//Configure the controller to make sure the robot drives straight
	MaintainAngleController.Reset();
	MaintainAngleController.SetSetpoint(0);
	MaintainAngleController.SetPercentTolerance(1);

	DistanceController.Reset();
	DistanceController.SetSetpoint(distance);
	DistanceController.SetPercentTolerance(1);

	MaintainAngleController.Enable();
	DistanceController.Enable();

	SmartDashboard::PutNumber("Target Distance", distance);

	//Wait until the robot reaches the target
	while(!DistanceController.OnTarget());
	{
		SmartDashboard::PutNumber("Current Distance", FrontLeftMotor.GetSelectedSensorPosition(0));
	}

	MaintainAngleController.Disable();
	DistanceController.Disable();
}

void Robot::TurnAngle(double angle)
{
	Gyro.Reset();

	//Remove the pointers since only one PID is being used
	DistancePID.SetAnglePID(nullptr);
	AnglePID.SetDistancePID(nullptr);

	AngleController.Reset();
	AngleController.SetSetpoint(angle);
	AngleController.SetPercentTolerance(1);

	AngleController.Enable();

	SmartDashboard::PutNumber("Target Angle", angle);

	//Wait until the robot reaches the target
	while(!AngleController.OnTarget())
	{
		SmartDashboard::PutNumber("Current Angle", Gyro.GetAngle());
	}
	AngleController.Disable();
}
