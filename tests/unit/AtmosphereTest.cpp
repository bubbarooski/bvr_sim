// The atmosphere is checked against the 1976 standard written out
// independently here in SI units, so a units slip in the English-unit
// implementation cannot cancel out. Agreement with JSBSim's own atmosphere
// is checked separately in PhysicsValidationTest, which can link JSBSim.

#include <cmath>

#include <gtest/gtest.h>

#include "sim/Atmosphere.h"

namespace {

constexpr double kFtToM = 0.3048;
constexpr double kKgPerM3ToSlugPerFt3 = 0.00194032;
constexpr double kPaToPsf = 0.0208854;
constexpr double kMPerSToFtPerS = 1.0 / kFtToM;

struct Si {
    double densityKgPerM3;
    double speedOfSoundMPerS;
    double temperatureK;
    double pressurePa;
};

// U.S. Standard Atmosphere 1976, first three layers, SI, geopotential.
Si Standard1976(double geometricAltitudeM, double offsetK) {
    constexpr double kR = 287.0531;
    constexpr double kG = 9.80665;
    constexpr double kEarthM = 6356766.0;
    double h = geometricAltitudeM * kEarthM / (kEarthM + geometricAltitudeM);

    struct Layer { double baseM, baseK, lapse; };
    const Layer layers[] = {{0.0, 288.15, -0.0065}, {11000.0, 216.65, 0.0}, {20000.0, 216.65, 0.001}};
    double p = 101325.0;
    int i = 0;
    for (; i < 2 && h >= layers[i + 1].baseM; ++i) {
        double t0 = layers[i].baseK + offsetK;
        double dh = layers[i + 1].baseM - layers[i].baseM;
        p = layers[i].lapse != 0.0 ? p * std::pow(t0 / (t0 + layers[i].lapse * dh), kG / (kR * layers[i].lapse))
                                   : p * std::exp(-kG * dh / (kR * t0));
    }
    double t0 = layers[i].baseK + offsetK;
    double dh = h - layers[i].baseM;
    double t = t0 + layers[i].lapse * dh;
    p = layers[i].lapse != 0.0 ? p * std::pow(t0 / t, kG / (kR * layers[i].lapse)) : p * std::exp(-kG * dh / (kR * t0));
    return {p / (kR * t), std::sqrt(1.4 * kR * t), t, p};
}

void ExpectMatches(double altitudeFt, double offsetC) {
    Atmosphere::AtmosphereState state = Atmosphere::Lookup(altitudeFt, offsetC);
    Si expected = Standard1976(altitudeFt * kFtToM, offsetC);
    SCOPED_TRACE(::testing::Message() << altitudeFt << " ft, ISA" << (offsetC >= 0 ? "+" : "") << offsetC);
    EXPECT_NEAR(state.densitySlugPerFt3 / (expected.densityKgPerM3 * kKgPerM3ToSlugPerFt3), 1.0, 1e-4);
    EXPECT_NEAR(state.speedOfSoundFtPerSec / (expected.speedOfSoundMPerS * kMPerSToFtPerS), 1.0, 1e-5);
    EXPECT_NEAR(state.temperatureR / (expected.temperatureK * 1.8), 1.0, 1e-5);
    EXPECT_NEAR(state.pressurePsf / (expected.pressurePa * kPaToPsf), 1.0, 1e-4);
}

}  // namespace

TEST(Atmosphere, SeaLevelMatchesStandardISA) {
    Atmosphere::AtmosphereState state = Atmosphere::Lookup(0.0);
    EXPECT_NEAR(state.densitySlugPerFt3, 0.0023769, 1e-7);
    EXPECT_NEAR(state.speedOfSoundFtPerSec, 1116.45, 0.01);
    EXPECT_NEAR(state.pressurePsf, 2116.228, 1e-6);
}

// Includes points midway between 5000 ft multiples (12 500, 27 500), where
// a linearly interpolated table of an exponential profile is worst, and
// points above 65 000 ft (70 000, 100 000), past where such tables stop.
TEST(Atmosphere, MatchesStandard1976AcrossLayers) {
    for (double altitudeFt : {0.0, 12500.0, 25000.0, 27500.0, 36089.0, 40000.0, 50000.0, 70000.0, 100000.0}) {
        ExpectMatches(altitudeFt, 0.0);
    }
}

// A hot or cold day shifts every layer's temperature and leaves sea-level
// pressure alone, so density aloft changes hydrostatically.
TEST(Atmosphere, TemperatureOffsetIsHydrostatic) {
    for (double offsetC : {-20.0, 15.0, 30.0}) {
        for (double altitudeFt : {0.0, 15000.0, 40000.0, 80000.0}) ExpectMatches(altitudeFt, offsetC);
    }
    Atmosphere::AtmosphereState standard = Atmosphere::Lookup(0.0);
    Atmosphere::AtmosphereState hot = Atmosphere::Lookup(0.0, 30.0);
    EXPECT_DOUBLE_EQ(hot.pressurePsf, standard.pressurePsf);
    EXPECT_LT(hot.densitySlugPerFt3, standard.densitySlugPerFt3);
    EXPECT_GT(hot.speedOfSoundFtPerSec, standard.speedOfSoundFtPerSec);
    // A hot column is taller: at altitude, hot air is DENSER than standard
    // air at the same height, because less of the mass sits below it.
    EXPECT_GT(Atmosphere::Lookup(40000.0, 30.0).densitySlugPerFt3, Atmosphere::Lookup(40000.0).densitySlugPerFt3);
}

// Repeated calls at alternating offsets must not leak one offset's cached
// pressure profile into another's result.
TEST(Atmosphere, OffsetCacheDoesNotLeak) {
    double standard = Atmosphere::Lookup(30000.0).densitySlugPerFt3;
    double hot = Atmosphere::Lookup(30000.0, 25.0).densitySlugPerFt3;
    EXPECT_DOUBLE_EQ(Atmosphere::Lookup(30000.0).densitySlugPerFt3, standard);
    EXPECT_DOUBLE_EQ(Atmosphere::Lookup(30000.0, 25.0).densitySlugPerFt3, hot);
}

TEST(Atmosphere, DensityFallsMonotonically) {
    double previous = Atmosphere::Lookup(-1000.0).densitySlugPerFt3;
    for (double altitudeFt = 0.0; altitudeFt <= 150000.0; altitudeFt += 500.0) {
        double density = Atmosphere::Lookup(altitudeFt).densitySlugPerFt3;
        EXPECT_LT(density, previous) << altitudeFt;
        previous = density;
    }
}
