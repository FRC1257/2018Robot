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
}

#endif
