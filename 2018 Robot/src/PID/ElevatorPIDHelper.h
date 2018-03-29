#ifndef TALON_PID_SOURCE
#define TALON_PID_SOURCE

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include "../Constants.h"

using namespace frc;

class ElevatorPIDHelper : public PIDSource, public PIDOutput
{
private:
	WPI_TalonSRX* m_TalonWithEncoder;
	Talon* m_FollowerMotor;
	static constexpr double m_DRUM_DIAMETER = 1.5;

public:
	ElevatorPIDHelper(WPI_TalonSRX* TalonWithEncoder, Talon* FollowerMotor);
	virtual ~ElevatorPIDHelper();
	double PIDGet() override;
	double GetHeightInches();
	void PIDWrite(double output) override;
};

#endif
