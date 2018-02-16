#ifndef CONSTANTS
#define CONSTANTS

namespace consts
{
	enum class AutoPosition
	{
		LEFT_START,
		MIDDLE_START,
		RIGHT_START,
		DEFAULT
	};

	enum class AutoObjective
	{
		SWITCH,
		SCALE,
		BASELINE,
		DEFAULT
	};

	enum class MiddleApproach
	{
		FRONT,
		SIDE
	};

	enum ElevatorIncrement
	{
		//These numbers are fillers until build attached the elevator
		GROUND,
		SWITCH,
		SCALE,
		MAXHEIGHT
	};

	constexpr int PIDLoopIdx = 0;
	constexpr int timeoutMs = 10;

	int height;
	int setpointDistance;

	constexpr int NUM_ELEVATOR_SETPOINTS = 5;
	constexpr double ELEVATOR_SETPOINTS[NUM_ELEVATOR_SETPOINTS] = {0, 40, 60, 90, 100};

	constexpr double PI = 3.1416;
	constexpr double WHEEL_DIAMETER = 6;
	constexpr double PULSES_PER_REV = 4096;

	constexpr double GAME_DATA_TIMOUT_S = 1;
}

#endif
