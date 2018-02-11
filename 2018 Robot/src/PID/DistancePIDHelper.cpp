#include "DistancePIDHelper.h"
#include "../Robot.h"

DistancePIDHelper::DistancePIDHelper(WPI_TalonSRX& motor, DifferentialDrive& driveTrain) :
	m_motor(motor),
	m_DriveTrain(driveTrain),
	m_output(0),
	m_AnglePID(nullptr)
{

}

DistancePIDHelper::~DistancePIDHelper()
{

}

double DistancePIDHelper::PIDGet()
{
	return PulsesToInches(m_motor.GetSelectedSensorPosition(0));
}

void DistancePIDHelper::PIDWrite(double output)
{
	SmartDashboard::PutNumber("Distance PID Output", output);
	double angle = m_AnglePID == nullptr ? 0 : m_AnglePID->GetOutput();
	m_DriveTrain.ArcadeDrive(output, angle);
	m_output = output;
}

double DistancePIDHelper::GetOutput()
{
	return m_output;
}

void DistancePIDHelper::SetAnglePID(AnglePIDOutput* anglePID)
{
	m_AnglePID = anglePID;
}
