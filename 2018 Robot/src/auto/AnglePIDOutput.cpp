#include <auto/AnglePIDOutput.h>

AnglePIDOutput::AnglePIDOutput(DifferentialDrive& driveTrain) : m_driveTrain(driveTrain)
{

}

AnglePIDOutput::~AnglePIDOutput()
{

}

void AnglePIDOutput::PIDWrite(double output)
{
	m_driveTrain.ArcadeDrive(0, output);
}
