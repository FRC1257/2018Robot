#ifndef CONSTANTS
#define CONSTANTS

namespace constants
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

	const int kPIDLoopIdx = 0;
	const int kTimeoutMs = 10;
}

#endif
