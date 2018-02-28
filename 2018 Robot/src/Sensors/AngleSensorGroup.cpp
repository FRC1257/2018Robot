#include "AngleSensorGroup.h"

AngleSensorGroup::AngleSensorGroup(SPI::Port navXPort, SPI::Port gyroPort) :
	m_NavX(navXPort),
	m_Gyro(gyroPort)
{

}

AngleSensorGroup::~AngleSensorGroup()
{

}

double AngleSensorGroup::PIDGet()
{
	return GetAngle();
}

void AngleSensorGroup::Reset()
{
	m_NavX.ZeroYaw();
	m_Gyro.Reset();
}

double AngleSensorGroup::GetAngle()
{
	if(m_NavX.IsConnected())
	{
		return m_NavX.GetYaw();
	}
	else
	{
		return m_Gyro.GetAngle();
	}
}

double AngleSensorGroup::GetDisplacementX()
{
	return m_NavX.GetDisplacementX() * 39.37;
}

double AngleSensorGroup::GetDisplacementZ()
{
	return m_NavX.GetDisplacementZ() * 39.37;
}

double AngleSensorGroup::GetDisplacementY()
{
	return m_NavX.GetDisplacementY() * 39.37;
}

void AngleSensorGroup::ResetDisplacement()
{
	m_NavX.ResetDisplacement();
}
