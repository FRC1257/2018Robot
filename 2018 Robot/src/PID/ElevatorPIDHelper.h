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

	static constexpr double DRUM_DIAMETER = 1.5;

public:
	ElevatorPIDHelper(WPI_TalonSRX* TalonWithEncoder);
	virtual ~ElevatorPIDHelper();
	double PIDGet() override;
	double GetHeightInches();
	void PIDWrite(double output) override;
};

#endif
