#ifndef CONSTANTS
#define CONSTANTS

namespace consts
{
	// Auto Constants
	constexpr double GAME_DATA_TIMEOUT_S = 1;
	constexpr double PID_TIMEOUT_S = 5;

	// Auto Sendable Chooser enums
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
		DEFAULT
	};

	enum class SwitchApproach
	{
		FRONT,
		SIDE
	};

	// Elevator increment enum for automatic mode:
	// - Because this is an enum and not an enum class, you can use
	//   the enum value as an array index for the ELEVATOR_SETPOINTS array
	enum ElevatorIncrement
	{
		GROUND,
		SCALE_LOW,
		SCALE_MID,
		SCALE_HIGH,
		MAX_HEIGHT
	};

	constexpr double DRIVE_SPEED_REDUCTION = 5. / 8.;

	// Elevator Constants
	constexpr int NUM_ELEVATOR_SETPOINTS = 5;
	constexpr double ELEVATOR_SETPOINTS[NUM_ELEVATOR_SETPOINTS] = {0, 20, 40, 60, 100};
	constexpr double ELEVATOR_INCREMENT_PER_CYCLE = 35. / 20.; // 35" per second
	constexpr double ELEVATOR_SPEED_REDUCTION = 1. / 3.;
	constexpr int ELEVATOR_CONT_CURRENT_MAX = 60;
	constexpr int ELEVATOR_CONT_CURRENT_TIMEOUT_MS = 2000;

	constexpr double ELEVATOR_PID_DEADBAND = 2.0;
	constexpr double ELEVATOR_PID_CONSTANTS_RISING[] = {0.25, 0., 0.};
	constexpr double ELEVATOR_PID_CONSTANTS_LOWERING[] = {0.25, 0., 0.};

	// Talon configuration constants
	constexpr int PID_LOOP_ID = 0;
	constexpr int TALON_TIMEOUT_MS = 10;

	// Current Limiting Constants
	constexpr int FORTY_AMP_FUSE_CONT_MAX = 50; // The continuous max current draw for a 40 amp breaker
	constexpr int THIRTY_AMP_FUSE_CONT_MAX = 35; // The continuous max current draw for a 30 amp breaker
	constexpr int CONT_CURRENT_TIMEOUT_MS = 500;

	// Encoder Constants
	constexpr double PI = 3.1416;
	constexpr double WHEEL_DIAMETER = 6;
	constexpr double PULSES_PER_REV = 4096;

	// PID Constants
	constexpr int PID_LOOP_X = 0;
	constexpr int TIMEOUT_MS = 10;

	// Intake Constants
	constexpr double MIN_DISTANCE_TO_CUBE = 9.0;
	constexpr double INTAKE_SPEED = 0.80;
	constexpr double INTAKE_SPEED_WHILE_TURNING = 0.5;
	constexpr double RESTING_INTAKE_SPEED = 0.05;
}

#endif
