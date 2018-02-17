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

	// Because this is an enum and not an enum class, you can use
	// the enum value as an array inded for the ELEVATOR_SETPOINTS array
	enum ElevatorIncrement
	{
		GROUND,
		SWITCH,
		MIDDLE,
		SCALE,
		MAXHEIGHT
	};

	constexpr int PID_LOOP_ID = 0;
	constexpr int TALON_TIMEOUT_MS = 10;

	//Numbers are fillers until build attaches the elevator
	constexpr int NUM_ELEVATOR_SETPOINTS = 5;
	constexpr double ELEVATOR_SETPOINTS[NUM_ELEVATOR_SETPOINTS] = {0, 40, 60, 90, 100};

	constexpr double PI = 3.1416;
	constexpr double WHEEL_DIAMETER = 6;
	constexpr double PULSES_PER_REV = 4096;

	constexpr double GAME_DATA_TIMOUT_S = 1;
}

#endif
