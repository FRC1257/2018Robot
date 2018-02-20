#include "StabilizedUltrasonic.h"

double average(std::deque<double>& array)
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
	// Can't use a reference because 'nth_element' would reorder the
	// original array
	size_t midPoint = array.size() / 2;
	nth_element(array.begin(), array.begin() + midPoint, array.end());
	return array[midPoint];
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
	return GetRangeInches();
}

bool StabilizedUltrasonic::IsEnabled()
{
	return DistanceSensor.IsEnabled();
}

// Return the median of the past few distance values
double StabilizedUltrasonic::GetRangeInches()
{
	// Delete the oldest measurement when the array of old distances
	// reaches max capacity
	if(m_prevDistances.size() == MAX_NUM_OF_DISTANCES)
	{
		m_prevDistances.pop_front();
	}
	m_prevDistances.push_back(DistanceSensor.GetRangeInches());

	// Use the median of the past few measurements as the current distance
	return median(m_prevDistances);
}
