#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include "sim/Rng.h"

// The stream is pinned to exact bit patterns so that a seed means the same
// run on every platform. The standard library's own distributions are
// implementation-defined and gave Windows and Linux different streams from
// the same seed; Rng now builds both distributions from correctly-rounded
// arithmetic alone, so these must match bit for bit everywhere. If this
// fails on one platform only, the stream has become platform-dependent
// again -- check for a new libm call or floating-point contraction.
TEST(Rng, StreamIsPinnedAcrossPlatforms) {
    Rng rng;
    rng.Seed(12345);
    EXPECT_EQ(rng.NextUniform(0.0, 1.0), 0x1.6e367c6f88882p-2);
    EXPECT_EQ(rng.NextUniform(-10.0, 10.0), -0x1.fdbbda5d6c0f0p+0);
    EXPECT_EQ(rng.NextGaussian(0.0, 1.0), 0x1.d537046ea9370p+0);
    EXPECT_EQ(rng.NextGaussian(5.0, 2.0), 0x1.6ceb1c49e7106p+2);
}

// The portable logarithm trades correct rounding for being identical
// everywhere; it must still be accurate to a few ulp across the whole
// range the polar method feeds it, (0, 1), and beyond.
TEST(Rng, PortableLogIsAccurate) {
    for (double x : {1e-300, 1e-100, 1e-10, 0.001, 0.1, 0.5, 0.70710678118654752440, 0.7071067811865476, 0.9,
                     0.999999, 1.0, 1.000001, 2.0, 10.0, 12345.678, 1e100}) {
        double expected = std::log(x);
        double tolerance = std::max(4e-16 * std::fabs(expected), 4e-16);
        EXPECT_NEAR(Rng::PortableLog(x), expected, tolerance) << x;
    }
    Rng rng;
    rng.Seed(5);
    for (int i = 0; i < 100000; ++i) {
        double x = rng.NextUniform(0.0, 1.0);
        if (x == 0.0) continue;
        double expected = std::log(x);
        EXPECT_NEAR(Rng::PortableLog(x), expected, std::max(4e-16 * std::fabs(expected), 4e-16)) << x;
    }
}

TEST(Rng, MomentsMatchTheDistributions) {
    Rng rng;
    rng.Seed(99);
    constexpr int kN = 200000;
    double sum = 0.0, sumSq = 0.0, uniformSum = 0.0;
    for (int i = 0; i < kN; ++i) {
        double g = rng.NextGaussian(0.0, 1.0);
        sum += g;
        sumSq += g * g;
        double u = rng.NextUniform(0.0, 1.0);
        EXPECT_GE(u, 0.0);
        EXPECT_LT(u, 1.0);
        uniformSum += u;
    }
    EXPECT_NEAR(sum / kN, 0.0, 0.01);
    EXPECT_NEAR(sumSq / kN, 1.0, 0.01);
    EXPECT_NEAR(uniformSum / kN, 0.5, 0.005);
}

// Every stochastic system draws from one seeded stream, so reproducing a
// run rests on the seed fully determining that stream.
TEST(Rng, SameSeedReproducesTheStream) {
    Rng a;
    Rng b;
    a.Seed(424242);
    b.Seed(424242);
    for (int i = 0; i < 20; ++i) {
        EXPECT_EQ(a.NextGaussian(0.0, 1.0), b.NextGaussian(0.0, 1.0));
        EXPECT_EQ(a.NextUniform(-10.0, 10.0), b.NextUniform(-10.0, 10.0));
    }
}

TEST(Rng, DifferentSeedsDiverge) {
    Rng a;
    Rng b;
    a.Seed(1);
    b.Seed(2);
    EXPECT_NE(a.NextGaussian(0.0, 1.0), b.NextGaussian(0.0, 1.0));
}

// A zero sigma is a legitimate "no jitter" choice and must not consume a
// draw, or turning one noise source off would shift every draw after it.
TEST(Rng, ZeroSigmaReturnsMeanWithoutDrawing) {
    Rng a;
    Rng b;
    a.Seed(7);
    b.Seed(7);
    EXPECT_EQ(a.NextGaussian(3.5, 0.0), 3.5);
    EXPECT_EQ(a.NextGaussian(0.0, 1.0), b.NextGaussian(0.0, 1.0));
}
