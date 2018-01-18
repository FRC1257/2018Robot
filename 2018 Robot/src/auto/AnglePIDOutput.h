#ifndef SRC_AUTO_ANGLEPIDOUTPUT_H_
#define SRC_AUTO_ANGLEPIDOUTPUT_H_

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
