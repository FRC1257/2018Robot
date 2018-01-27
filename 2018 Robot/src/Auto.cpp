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

void Robot::DriveFor(double distance)
{
	//FrontLeft is placeholder until we learn which motor has an encoder
	double targetDistance = distance + PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0));

	NavX.Reset();

	AnglePID.SetActive(false); //Make sure only the distance controller is driving to avoid conflicts
	AngleController.Reset();
	AngleController.SetSetpoint(0);
	AngleController.SetPercentTolerance(5);

	DistanceController.Reset();
	DistanceController.SetSetpoint(targetDistance);
	DistanceController.SetPercentTolerance(5);

	AngleController.Enable();
	DistanceController.Enable();

//	while(!DistanceController.OnTarget()); //Wait until the robot reaches the target
//	AngleController.Disable();
//	DistanceController.Disable();
}

void Robot::TurnAngle(double angle)
{
	NavX.Reset();

	AnglePID.SetActive(true);
	AngleController.Reset();
	AngleController.SetSetpoint(angle);
	AngleController.SetPercentTolerance(5);

	AngleController.Enable();

//	while(!AngleController.OnTarget()); //Wait until the robot reaches the target
//	AngleController.Disable();
}
