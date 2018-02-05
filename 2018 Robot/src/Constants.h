#ifndef CONSTANTS
#define CONSTANTS

namespace consts
{
	enum class AutoPosition
	{
		LEFT_START,
		MIDDLE_START,
		RIGHT_START
	};

	enum class AutoObjective
	{
		SWITCH,
		SCALE,
		BASELINE,
		STAY_STILL
	};

	const int PIDLoopIdx = 0;
	const int timeoutMs = 10;

	double PI = 3.1416;
	double WHEEL_DIAMETER = 6;
	double PULSES_PER_REV = 4096;
}

#endif
