#include "auto/DistancePIDHelper.h"
#include "Robot.h"

DistancePIDHelper::DistancePIDHelper(WPI_TalonSRX& motor, DifferentialDrive& driveTrain, AnglePIDOutput& anglePID) :
	m_motor(motor),
	m_driveTrain(driveTrain),
	m_anglePID(anglePID)
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
	m_driveTrain.ArcadeDrive(output, m_anglePID.GetOutput());
}
