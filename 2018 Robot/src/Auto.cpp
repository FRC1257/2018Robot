#include "Robot.h"
#include "auto/AnglePIDOutput.h"

void Robot::AutonomousInit()
{
	SmartDashboard::PutString("DB/String 0", "Robot At Left Position");
	SmartDashboard::PutString("DB/String 1", "Robot At Middle Position");
	SmartDashboard::PutString("DB/String 2", "Robot At Right Position");

	std::string gameData;
	void DriverStation::WaitForData();

	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

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

void Robot::DriveFor(double distance, double speed)
{
	double TargetDistance = distance + PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0));
	while(PulsesToInches(FrontLeftMotor.GetSelectedSensorPosition(0)) < TargetDistance) //Not sure which motor to test the encoder for - Omay
		{
			DriveTrain.ArcadeDrive(speed, 0);
		}

	DriveTrain.ArcadeDrive(0, 0);
}

void Robot::TurnAngle(double angle)
{
	Gyro.Reset();

	angleController.Reset();
	angleController.SetSetpoint(angle);
	angleController.SetPercentTolerance(5);

	angleController.Enable();
}
