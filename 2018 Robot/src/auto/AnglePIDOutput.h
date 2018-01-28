#ifndef ANGLE_PID_OUTPUT
#define ANGLE_PID_OUTPUT

#include <WPILib.h>

using namespace frc;

class DistancePIDHelper; //Forward declare the DistancePIDHelper class to avoid circular header dependencies

class AnglePIDOutput : public PIDOutput
{
private:
	DifferentialDrive& m_driveTrain;
	double m_output;
	DistancePIDHelper* m_distancePID;

public:
	AnglePIDOutput(DifferentialDrive& DriveTrain);
	virtual ~AnglePIDOutput();

	void PIDWrite(double output) override;

	double GetOutput();
	void SetDistancePID(DistancePIDHelper* distancePID);
};

#endif
