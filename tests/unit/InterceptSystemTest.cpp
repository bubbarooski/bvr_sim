#include <cmath>

#include <gtest/gtest.h>

#include "systems/InterceptSystem.h"

TEST(Pk, PassThroughTheAirframeIsACertainKill) {
    EXPECT_DOUBLE_EQ(InterceptSystem::Pk(0.0, 15.0, 5.0), 1.0);
    EXPECT_DOUBLE_EQ(InterceptSystem::Pk(0.0, 12.0, 5.0), 1.0);
}

TEST(Pk, AtKillRadiusIsHalfOfTheRescale) {
    // 0.5 of the bare logistic, divided by its value at zero.
    EXPECT_NEAR(InterceptSystem::Pk(15.0, 15.0, 5.0), 0.5 * (1.0 + std::exp(-3.0)), 1e-12);
}

TEST(Pk, MatchesReferenceFigure) {
    // REFERENCE.md 12.4: d_miss=10ft, r_kill=15ft, w_falloff=5ft -> 0.7675.
    EXPECT_NEAR(InterceptSystem::Pk(10.0, 15.0, 5.0), 0.7675, 1e-4);
}

TEST(Pk, MonotonicallyDecreasesWithMissDistance) {
    double prev = InterceptSystem::Pk(0.0, 15.0, 5.0);
    for (double missFt = 1.0; missFt <= 60.0; missFt += 1.0) {
        double pk = InterceptSystem::Pk(missFt, 15.0, 5.0);
        EXPECT_LT(pk, prev);
        prev = pk;
    }
}

TEST(Pk, ApproachesZeroAndOneAtExtremes) {
    EXPECT_DOUBLE_EQ(InterceptSystem::Pk(-1000.0, 15.0, 5.0), 1.0);  // never above 1
    EXPECT_NEAR(InterceptSystem::Pk(1000.0, 15.0, 5.0), 0.0, 1e-6);
}
