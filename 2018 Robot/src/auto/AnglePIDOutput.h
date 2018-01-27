#ifndef ANGLE_PID_OUTPUT
#define ANGLE_PID_OUTPUT

#include <WPILib.h>

using namespace frc;

class AnglePIDOutput : public PIDOutput
{
private:
	DifferentialDrive& m_driveTrain;

public:
	AnglePIDOutput(DifferentialDrive& DriveTrain);
	virtual ~AnglePIDOutput();

	void PIDWrite(double output) override;
};

#endif
