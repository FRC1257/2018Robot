#ifndef ANGLE_SENSOR
#define ANGLE_SENSOR

#include <WPILib.h>
#include <AHRS.h>

using namespace frc;

class AngleSensorGroup : public PIDSource
{
private:
	AHRS m_NavX;
	ADXRS450_Gyro m_Gyro;

public:
	AngleSensorGroup(SPI::Port navXPort, SPI::Port gyroPort);
	virtual ~AngleSensorGroup();

	double PIDGet();

	void Reset();
	double GetAngle();

	double GetDisplacementX();
	double GetDisplacementZ();
	double GetDisplacementY();
	void ResetDisplacement();
};

#endif
