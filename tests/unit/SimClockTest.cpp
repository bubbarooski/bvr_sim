// The viewer's speed controls set the run speed: while paused they change
// only what Resume returns to, and never start the clock.

#include <cmath>

#include <gtest/gtest.h>

#include "sim/SimClock.h"

TEST(SimClock, RunScaleWhilePausedIsWhatResumeReturnsTo) {
    SimClock clock;
    clock.StartPaused(2.0);
    EXPECT_TRUE(clock.Paused());
    EXPECT_EQ(clock.RunScale(), 2.0);

    clock.SetRunScale(50.0);
    EXPECT_TRUE(clock.Paused());
    EXPECT_EQ(clock.RunScale(), 50.0);

    clock.Resume();
    EXPECT_EQ(clock.timeScale, 50.0);
}

TEST(SimClock, SetRunScaleWhileRunningChangesTheLiveScale) {
    SimClock clock;
    clock.SetRunScale(10.0);
    EXPECT_FALSE(clock.Paused());
    EXPECT_EQ(clock.timeScale, 10.0);

    clock.SetRunScale(0.0);  // ignored: that would be Pause
    EXPECT_EQ(clock.timeScale, 10.0);

    clock.Pause();
    clock.Resume();
    EXPECT_EQ(clock.timeScale, 10.0);
}

// --- The accumulator: how many fixed steps a stretch of real time buys.

namespace {

int StepsAfter(SimClock& clock, double realSeconds) {
    clock.Advance(realSeconds);
    int steps = 0;
    while (clock.ShouldStep()) {
        EXPECT_EQ(clock.ConsumeStep(), SimClock::kFixedDt);  // the step never changes size
        ++steps;
    }
    return steps;
}

}  // namespace

TEST(SimClock, ScaleMultipliesHowManyStepsRunNotTheirSize) {
    SimClock one, two, half;
    two.SetScale(2.0);
    half.SetScale(0.5);
    // 0.1 s of real time is 12 fixed steps of sim time at 1x.
    EXPECT_EQ(StepsAfter(one, 0.1), 12);
    EXPECT_EQ(StepsAfter(two, 0.1), 24);
    EXPECT_EQ(StepsAfter(half, 0.1), 6);
}

TEST(SimClock, APausedClockBuysNoSteps) {
    SimClock clock;
    clock.Pause();
    EXPECT_EQ(StepsAfter(clock, 1.0), 0);
}

// A debugger pause or a window drag must not queue up minutes of catch-up.
TEST(SimClock, AStallIsClampedToAQuarterSecond) {
    SimClock clock;
    EXPECT_EQ(StepsAfter(clock, 30.0), 30);  // 0.25 s * 120 steps/s
}

// A negative scale would run the accumulator backwards and never step again.
TEST(SimClock, ANegativeOrNonFiniteScaleIsIgnored) {
    SimClock clock;
    clock.SetScale(3.0);
    clock.SetScale(-1.0);
    EXPECT_EQ(clock.timeScale, 3.0);
    clock.SetScale(std::nan(""));
    EXPECT_EQ(clock.timeScale, 3.0);
    clock.SetRunScale(-5.0);
    EXPECT_EQ(clock.timeScale, 3.0);
    clock.StartPaused(-2.0);  // resume to 1x, as for 0
    clock.Resume();
    EXPECT_EQ(clock.timeScale, 1.0);
}

// When stepping cannot keep up (a Debug build at 100x), the backlog is
// dropped rather than worked off at the cost of the whole frame: the sim runs
// slower than asked and the window stays responsive.
TEST(SimClock, DroppingTheBacklogLeavesNothingToStep) {
    SimClock clock;
    clock.SetScale(100.0);
    clock.Advance(0.25);
    ASSERT_TRUE(clock.ShouldStep());
    clock.DropBacklog();
    EXPECT_FALSE(clock.ShouldStep());
    clock.Advance(1.0 / 120.0);  // and it runs normally afterwards
    EXPECT_TRUE(clock.ShouldStep());
}
