#ifndef DISTANCE_PID
#define DISTANCE_PID

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include "AnglePIDOutput.h"

using namespace frc;

class DistancePIDHelper : public PIDSource, public PIDOutput
{
private:
	WPI_TalonSRX& m_motor;
	DifferentialDrive& m_driveTrain;
	AnglePIDOutput& m_anglePID;

public:
	DistancePIDHelper(WPI_TalonSRX& motor, DifferentialDrive& driveTrain, AnglePIDOutput& anglePID);
	virtual ~DistancePIDHelper();

	double PIDGet() override;
	void PIDWrite(double output) override;
};

#endif
