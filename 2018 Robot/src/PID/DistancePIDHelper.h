#ifndef DISTANCE_PID
#define DISTANCE_PID

#include <WPILib.h>
#include <ctre/Phoenix.h>

using namespace frc;

class AnglePIDOutput; //Forward declare the AnglePIDOutput class to avoid circular header dependencies

class DistancePIDHelper : public PIDSource, public PIDOutput
{
private:
	WPI_TalonSRX& m_motor;
	DifferentialDrive& m_DriveTrain;
	double m_output;
	AnglePIDOutput* m_AnglePID;

public:
	DistancePIDHelper(WPI_TalonSRX& motor, DifferentialDrive& driveTrain);
	virtual ~DistancePIDHelper();

	double PIDGet() override;
	void PIDWrite(double output) override;

	double GetOutput();
	void SetAnglePID(AnglePIDOutput* anglePID);
};

#endif
