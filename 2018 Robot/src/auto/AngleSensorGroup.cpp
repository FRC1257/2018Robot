#include "auto/AngleSensorGroup.h"

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
