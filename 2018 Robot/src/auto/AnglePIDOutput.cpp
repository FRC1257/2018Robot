#include "auto/AnglePIDOutput.h"

AnglePIDOutput::AnglePIDOutput(DifferentialDrive& driveTrain) :
	m_driveTrain(driveTrain),
	m_output(0),
	m_active(true)
{

}

AnglePIDOutput::~AnglePIDOutput()
{

}

void AnglePIDOutput::PIDWrite(double output)
{
	if(m_active) m_driveTrain.ArcadeDrive(0, output);
	m_output = output;
}

double AnglePIDOutput::GetOutput()
{
	return m_output;
}

void AnglePIDOutput::SetActive(bool active)
{
	m_active = active;
}
