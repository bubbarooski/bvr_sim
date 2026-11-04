#include <gtest/gtest.h>

#include "util/Vec3.h"

// The two Eigen behaviours the sim relies on: a default Vec3 is the origin
// (EIGEN_INITIALIZE_MATRICES_BY_ZERO), and normalizing a zero vector gives
// zero rather than NaN.

TEST(Vec3, DefaultConstructsToZero) {
    Vec3 v;
    EXPECT_EQ(v, Vec3(0.0, 0.0, 0.0));
}

TEST(Vec3, NormalizeOfZeroVectorReturnsZero) {
    EXPECT_EQ(Vec3::Zero().normalized(), Vec3::Zero());
}
