#pragma once

// Shared tolerances and sentinels.
namespace Numeric {

// A vector part this small relative to the whole is rounding, not signal:
// below it a direction is undefined and treated as zero.
constexpr double kNegligibleFraction = 1e-9;

// Slack when comparing sim times built by summing the fixed step, so
// accumulated rounding cannot push a comparison one tick late.
constexpr double kTimeSlackS = 1e-9;

// Whether `period` seconds have passed since `last`, for every periodic
// process (radar frames, seeker sweeps, RWR sweeps, datalink updates, the
// fire-control delay). The clock is a sum of 1/120 s steps, which is not
// representable: twelve of them add to a hair under 0.1, so `now - last >=
// period` alone waits a thirteenth and a nominal 10 Hz process runs at 9.2.
inline bool PeriodElapsed(double now, double last, double period) { return now - last >= period - kTimeSlackS; }

// "Last happened" time for something that never has: far enough in the past
// that the first periodic check after spawn always fires.
constexpr double kLongAgoS = -1e18;

}  // namespace Numeric
