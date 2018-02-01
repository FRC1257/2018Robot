#include "auto/DistancePIDHelper.h"
#include "auto/AnglePIDOutput.h"
#include "Robot.h"

DistancePIDHelper::DistancePIDHelper(WPI_TalonSRX& motor, DifferentialDrive& driveTrain) :
	m_motor(motor),
	m_driveTrain(driveTrain),
	m_output(0),
	m_anglePID(nullptr)
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
	SmartDashboard::PutNumber("Distance Output", output);
	double angle = m_anglePID == nullptr ? 0 : m_anglePID->GetOutput();
	m_driveTrain.ArcadeDrive(0.2 + output, angle);
	m_output = output;
}

double DistancePIDHelper::GetOutput()
{
	return m_output;
}

void DistancePIDHelper::SetAnglePID(AnglePIDOutput* anglePID)
{
	m_anglePID = anglePID;
}
