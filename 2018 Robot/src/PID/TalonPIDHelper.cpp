#include <PID/TalonPIDHelper.h>
#include <Robot.h>

TalonPIDHelper::TalonPIDHelper(WPI_TalonSRX* TalonWithEncoder) :
	m_TalonWithEncoder(TalonWithEncoder)
{

}

TalonPIDHelper::~TalonPIDHelper()
{

}

double TalonPIDHelper::PIDGet()
{
	return PulsesToInches(m_TalonWithEncoder->GetSelectedSensorPosition(0));
}

void TalonPIDHelper::PIDWrite(double output)
{
	m_TalonWithEncoder->Set(output);
}
