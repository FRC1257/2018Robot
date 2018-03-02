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
	double height = GetHeightInches();
	SmartDashboard::PutNumber("Elevator Height", height);
	return height;
}

double ElevatorPIDHelper::GetHeightInches()
{
	double circumference = DRUM_DIAMETER * consts::PI;
	double revolutions = m_TalonWithEncoder->GetSelectedSensorPosition(0) / consts::PULSES_PER_REV;
	double distance = revolutions * circumference;

	return distance;
}

void ElevatorPIDHelper::PIDWrite(double output)
{
	SmartDashboard::PutNumber("Elevator PID Output", output);
	m_TalonWithEncoder->Set(output);
}
