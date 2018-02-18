#include "StabilizedUltrasonic.h"

double average(std::deque<double> array)
{
	double avg = 0;
	for(unsigned int i = 0; i < array.size(); i++)
	{
		avg += array[i];
	}
	avg /= array.size();

	return avg;
}

double median(std::deque<double> array)
{
	// Should switch over to median when possible
}

StabilizedUltrasonic::StabilizedUltrasonic(int pingChannel, int echoChannel) :
	DistanceSensor(pingChannel, echoChannel)
{

}

StabilizedUltrasonic::~StabilizedUltrasonic()
{

}

double StabilizedUltrasonic::PIDGet()
{
	return GetDistance();
}

// Return an average of the past
double StabilizedUltrasonic::GetDistance()
{
	// Delete the oldest measurement when the array of old distances
	// reaches max capacity
	if(m_prevDistances.size() == MAX_NUM_OF_DISTANCES)
	{
		m_prevDistances.pop_front();
	}
	m_prevDistances.push_back(DistanceSensor.GetRangeInches());

	// Use the average of the past few measurements as the current distance
	return average(m_prevDistances);
}
