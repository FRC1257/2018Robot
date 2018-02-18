#ifndef STABILIZED_ULTRASONIC
#define STABILIZED_ULTRASONIC

#include <WPILib.h>
#include <ctre/Phoenix.h>
#include <cmath>
#include <deque>

using namespace frc;

class StabilizedUltrasonic : public PIDSource
{
private:
	Ultrasonic DistanceSensor;
	static constexpr int MAX_NUM_OF_DISTANCES = 10;
	std::deque<double> m_prevDistances;

public:
	StabilizedUltrasonic(int pingChannel, int echoChannel);
	virtual ~StabilizedUltrasonic();
	double PIDGet() override;
	double GetDistance();
};

#endif
