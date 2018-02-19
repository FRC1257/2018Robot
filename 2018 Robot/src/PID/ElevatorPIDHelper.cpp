#include <PID/ElevatorPIDHelper.h>
#include <Robot.h>

ElevatorPIDHelper::ElevatorPIDHelper(WPI_TalonSRX* TalonWithEncoder) :
	m_TalonWithEncoder(TalonWithEncoder)
{

}

ElevatorPIDHelper::~ElevatorPIDHelper()
{

}

double ElevatorPIDHelper::PIDGet()
{
	return PulsesToInches(m_TalonWithEncoder->GetSelectedSensorPosition(0));
}

void ElevatorPIDHelper::PIDWrite(double output)
{
	m_TalonWithEncoder->Set(output);
}
