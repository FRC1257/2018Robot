#ifndef SRC_CONSTANTS_H_
#define SRC_CONSTANTS_H_

namespace consts
{
	// Current Limiting Constants
	constexpr int FOURTY_AMP_FUSE_CONT_MAX = 50; // The contintuous max current draw for a 40 amp breaker
	constexpr int THIRTY_AMP_FUSE_CONT_MAX = 35; // The contintuous max current draw for a 30 amp breaker
	constexpr int CONT_CURRENT_TIMEOUT_MS = 500;

	constexpr int NUM_OF_ELEVATOR_SETPOINTS = 5;
	constexpr double EVELVATOR_SETPOINTS[NUM_OF_ELEVATOR_SETPOINTS] = {0, 4, 8, 12, 16};
}

#endif /* SRC_CONSTANTS_H_ */
