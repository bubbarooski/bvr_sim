// Periodic processes run on a clock built by summing the fixed step, 1/120 s,
// which is not representable: after 12 steps the sum is a hair under 0.1, so
// a gate written as `now - last >= period` waits a 13th. A nominal 10 Hz
// sensor then fires every 12.5 ticks on average (9.6 Hz), unevenly.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "sim/SimClock.h"
#include "util/Numeric.h"

namespace {

// Ticks between successive firings of a gate on the real accumulated clock.
std::vector<int> TicksBetweenFirings(double periodS, double durationS) {
    std::vector<int> gaps;
    double now = 0.0, last = Numeric::kLongAgoS;
    int sinceLast = 0;
    bool first = true;
    while (now < durationS) {
        now += SimClock::kFixedDt;  // exactly what SimLoop::Step does
        ++sinceLast;
        if (Numeric::PeriodElapsed(now, last, periodS)) {
            if (!first) gaps.push_back(sinceLast);
            first = false;
            last = now;
            sinceLast = 0;
        }
    }
    return gaps;
}

}  // namespace

TEST(PeriodElapsed, EveryPeriodIsAWholeNumberOfTicksEveryTime) {
    // period (s), ticks it should take at 120 Hz
    const struct { double periodS; int ticks; } cases[] = {
        {0.1, 12}, {0.5, 60}, {1.0, 120}, {4.4, 528}, {0.0166666666667, 2},
    };
    for (const auto& c : cases) {
        std::vector<int> gaps = TicksBetweenFirings(c.periodS, 600.0);
        ASSERT_GT(gaps.size(), 10u) << c.periodS;
        for (size_t i = 0; i < gaps.size(); ++i) {
            ASSERT_EQ(gaps[i], c.ticks) << "period " << c.periodS << " s, firing " << i;
        }
    }
}

TEST(PeriodElapsed, FiresOnTheFirstCallAfterSpawn) {
    EXPECT_TRUE(Numeric::PeriodElapsed(0.0083, Numeric::kLongAgoS, 4.4));
}

TEST(PeriodElapsed, DoesNotFireEarly) {
    // One tick short of the period is not enough, slack or no slack.
    EXPECT_FALSE(Numeric::PeriodElapsed(0.1 - SimClock::kFixedDt, 0.0, 0.1));
    EXPECT_TRUE(Numeric::PeriodElapsed(0.1, 0.0, 0.1));
}
