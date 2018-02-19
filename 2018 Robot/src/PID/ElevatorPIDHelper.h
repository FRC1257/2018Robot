#ifndef TALON_PID_SOURCE
#define TALON_PID_SOURCE

#include <WPILib.h>
#include <ctre/Phoenix.h>

using namespace frc;

class ElevatorPIDHelper : public PIDSource, public PIDOutput
{
private:
	WPI_TalonSRX* m_TalonWithEncoder;

public:
	ElevatorPIDHelper(WPI_TalonSRX* TalonWithEncoder);
	virtual ~ElevatorPIDHelper();
	double PIDGet() override;
	void PIDWrite(double output) override;
};

#endif
