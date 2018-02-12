#ifndef SRC_constexprANTS_H_
#define SRC_constexprANTS_H_

namespace constexprs
{
	constexpr int CONTINUOUSAMPS = 10; // TODO Check values
	constexpr int TIMEOUTMS = 500;
	constexpr int PEAKAMPS = 15; // TODO Check values
	constexpr int DURATIONSMS = 100; // TODO Check values
	constexpr double STEPVALS[5] = {0, 4, 8, 12, 16};
	constexpr int CCL_SLOT_IDX = 0;
	constexpr int CCL_PID_LOOP_IDX = 0;
	constexpr int CCL_TIMEOUT_MS = 10;

	constexpr double CIM_REV_PER_MIN = 2670;
	constexpr double TOUGH_BOX_GEAR_RATIO = 12.75;
	constexpr double MAX_REV_PER_MS = CIM_REV_PER_MIN / TOUGH_BOX_GEAR_RATIO / 60000;
	constexpr double TICKS_PER_REV = 4096;
	constexpr double MAX_TICKS_PER_MS =  MAX_REV_PER_MS * TICKS_PER_REV;
	constexpr double PID_LOOPTIME_MS = 100;
	constexpr double MAX_VELOCITY = MAX_TICKS_PER_MS / PID_LOOPTIME_MS; //TODO set value
}

#endif /* SRC_constexprANTS_H_ */
