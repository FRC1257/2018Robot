#ifndef ANGLE_PID_OUTPUT
#define ANGLE_PID_OUTPUT

#include <WPILib.h>

using namespace frc;

class AnglePIDOutput : public PIDOutput
{
private:
	DifferentialDrive& m_driveTrain;
	double m_output;
	bool m_active;

public:
	AnglePIDOutput(DifferentialDrive& DriveTrain);
	virtual ~AnglePIDOutput();

	void PIDWrite(double output) override;

	double GetOutput();
	void SetActive(bool active);
};

#endif
