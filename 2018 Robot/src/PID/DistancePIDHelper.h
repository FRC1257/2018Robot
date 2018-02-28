#ifndef DISTANCE_PID
#define DISTANCE_PID

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <cmath>
#include "../Sensors/AngleSensorGroup.h"

using namespace frc;

class AnglePIDOutput; //Forward declare the AnglePIDOutput class to avoid circular header dependencies

class DistancePIDHelper : public PIDSource, public PIDOutput
{
private:
	WPI_TalonSRX& m_motor;
	DifferentialDrive& m_DriveTrain;
	double m_output;                 // Stores the motor output so that other classes can access it
	AnglePIDOutput* m_AnglePID;
	AngleSensorGroup& m_Gyro;
	double m_prevDistance;
	double m_distanceTraveled;

public:
	DistancePIDHelper(WPI_TalonSRX& motor, DifferentialDrive& driveTrain, AngleSensorGroup& gyro);
	virtual ~DistancePIDHelper();

	double PIDGet() override;
	void PIDWrite(double output) override;

	double GetOutput();
	void SetAnglePID(AnglePIDOutput* anglePID);

	void ResetPrevOutput();
};

#endif
