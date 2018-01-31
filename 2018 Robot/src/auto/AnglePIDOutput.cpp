#include "auto/AnglePIDOutput.h"
#include "auto/DistancePIDHelper.h"

AnglePIDOutput::AnglePIDOutput(DifferentialDrive& driveTrain) :
	m_driveTrain(driveTrain),
	m_output(0),
	m_distancePID(nullptr)
{

}

AnglePIDOutput::~AnglePIDOutput()
{

}

void AnglePIDOutput::PIDWrite(double output)
{
	SmartDashboard::PutNumber("Angle Output", output);
	double drive = m_distancePID == nullptr ? 0 : m_distancePID->GetOutput();
	m_driveTrain.ArcadeDrive(drive, output);
	m_output = output;
}

double AnglePIDOutput::GetOutput()
{
	return m_output;
}

void AnglePIDOutput::SetDistancePID(DistancePIDHelper* distancePID)
{
	m_distancePID = distancePID;
}
