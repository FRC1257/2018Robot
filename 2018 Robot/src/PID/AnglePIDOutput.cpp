#include "AnglePIDOutput.h"
#include "DistancePIDHelper.h"

AnglePIDOutput::AnglePIDOutput(DifferentialDrive& driveTrain) :
	m_driveTrain(driveTrain),
	m_output(0),
	m_distancePID(nullptr),
	m_testDistOutput(0)
{

}

AnglePIDOutput::~AnglePIDOutput()
{

}

void AnglePIDOutput::PIDWrite(double output)
{
	SmartDashboard::PutNumber("Angle Output", output);

	double drive = m_distancePID == nullptr ? 0 : m_distancePID->GetOutput();
	if(m_testDistOutput != 0) drive = m_testDistOutput;

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

void AnglePIDOutput::SetTestDistOutput(double testDistOutput)
{
	m_testDistOutput = testDistOutput;
}
