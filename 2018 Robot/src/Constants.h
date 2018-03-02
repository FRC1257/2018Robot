#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

namespace consts
{
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

	// Elevator Constants
	constexpr int NUM_ELEVATOR_SETPOINTS = 5;
	constexpr double ELEVATOR_SETPOINTS[NUM_ELEVATOR_SETPOINTS] = {3, 20, 40, 60, 70};
	constexpr double ELEVATOR_INCREMENT_PER_CYCLE = 35. / 20.; // 35" per second
	constexpr double ELEVATOR_SPEED_REDUCTION = 1 / 3;

	constexpr double MIN_DISTANCE_TO_CUBE = 9.0;
	constexpr double INTAKE_SPEED = 0.7;
}

#endif /* SRC_CONSTANTS_H_ */
