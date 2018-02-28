#include "DistancePIDHelper.h"
#include "../Robot.h"

DistancePIDHelper::DistancePIDHelper(WPI_TalonSRX& motor, DifferentialDrive& driveTrain, AngleSensorGroup& gyro) :
	m_motor(motor),
	m_DriveTrain(driveTrain),
	m_output(0),
	m_AnglePID(nullptr),
	m_Gyro(gyro),
	m_prevDistance(0),
	m_distanceTraveled(0)
{

}

DistancePIDHelper::~DistancePIDHelper()
{

}

double DistancePIDHelper::PIDGet()
{
	double changeInDistance = -PulsesToInches(m_motor.GetSelectedSensorPosition(0)) - m_prevDistance;
	double angleError = m_Gyro.GetAngle();
	double scaleFactor = 1;
	if(angleError > 1)
	{
		 scaleFactor /= 20 * m_Gyro.GetAngle();
	}
	m_distanceTraveled += changeInDistance * scaleFactor;
	m_prevDistance = m_distanceTraveled;
	return m_distanceTraveled;
}

void DistancePIDHelper::PIDWrite(double output)
{
	SmartDashboard::PutNumber("Distance PID Output", output);
	double angle = m_AnglePID == nullptr ? 0 : m_AnglePID->GetOutput();

	m_DriveTrain.ArcadeDrive(output, angle, false);
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

void DistancePIDHelper::ResetPrevOutput()
{
	m_prevDistance = 0;
}
