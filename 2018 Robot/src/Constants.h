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
		BASELINE
	};

	const int PIDLoopIdx = 0;
	const int timeoutMs = 10;

	const double PI = 3.1416;
	const double WHEEL_DIAMETER = 6;
	const double PULSES_PER_REV = 4096;
}

#endif
