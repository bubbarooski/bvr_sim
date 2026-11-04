// The radar's per-look measurement error and its track filter. The error
// laws are checked against the figures they come from (MIT Lincoln
// Laboratory's 10:1 monopulse beam splitting at 13 dB; the F-16 array's
// beamwidths), and the filter by what a Kalman filter must do: when its
// model is the truth, its stated covariance matches its actual error (mean
// normalized estimation error squared = the state dimension), and with a
// maneuver allowance it still smooths several looks into a better estimate
// than any one of them.

#include <cmath>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "sim/RadarTrackFilter.h"
#include "sim/ReceiverBudget.h"
#include "sim/Rng.h"
#include "util/Units.h"

namespace {

using Units::kFeetPerMeter;
using Units::kRadToDeg;

// One noisy look at truePosFt from the origin, drawn the way
// AirborneRadarSystem draws it.
Vec3 Measure(Rng& rng, const Vec3& truePosFt, const RadarTrackFilter::LookSigmas& sigmas) {
    double rangeFt = truePosFt.norm();
    Vec3 los = truePosFt * (1.0 / rangeFt);
    Vec3 az, el;
    RadarTrackFilter::MeasurementAxes(los, az, el);
    Vec3 m = truePosFt;
    m += los * rng.NextGaussian(0.0, sigmas.rangeM * kFeetPerMeter);
    m += az * rng.NextGaussian(0.0, sigmas.azimuthRad * rangeFt);
    m += el * rng.NextGaussian(0.0, sigmas.elevationRad * rangeFt);
    return m;
}

// A target 50 km out, crossing at 250 m/s, tracked once a second for
// `sweeps` sweeps. Returns the final estimate; truth is written out.
TrackEstimate Track(Rng& rng, int sweeps, double accelSigmaFtPerSec2, const RadarTrackFilter::LookSigmas& sigmas,
                    Vec3& truePosOut, Vec3& trueVelOut) {
    const Vec3 start{50000.0 * kFeetPerMeter, 2000.0, 0.0};
    const Vec3 vel{0.0, 0.0, 250.0 * kFeetPerMeter};
    TrackEstimate e;
    for (int i = 0; i < sweeps; ++i) {
        double t = static_cast<double>(i);
        Vec3 truth = start + vel * t;
        RadarTrackFilter::Predict(e, t, accelSigmaFtPerSec2);
        Vec3 m = Measure(rng, truth, sigmas);
        RadarTrackFilter::Update(e, t, m, truth.normalized(), truth.norm(), sigmas, 2000.0);
        truePosOut = truth;
    }
    trueVelOut = vel;
    return e;
}

}  // namespace

TEST(RadarTrackFilter, AngleErrorSplitsTheBeamTenToOneAt13Db) {
    // LL Parameter Estimation and Tracking, slide 27: ~0.10 of a beamwidth
    // at a 13 dB detection threshold.
    RadarTrackFilter::LookSigmas s = RadarTrackFilter::MeasurementSigmas(std::pow(10.0, 1.3), 1.0, 1.0, 1.6, 1e-6);
    EXPECT_NEAR(s.azimuthRad, 0.0990, 5e-4);
    EXPECT_NEAR(s.elevationRad, s.azimuthRad, 1e-15);
    // c tau / (2 sqrt(2 SNR)): 1 us is 150 m of range resolution.
    EXPECT_NEAR(s.rangeM, 149.896229 / std::sqrt(2.0 * std::pow(10.0, 1.3)), 1e-6);
    // Error falls as 1 / sqrt(SNR): 20 dB more, ten times smaller.
    RadarTrackFilter::LookSigmas s33 = RadarTrackFilter::MeasurementSigmas(std::pow(10.0, 3.3), 1.0, 1.0, 1.6, 1e-6);
    EXPECT_NEAR(s.azimuthRad / s33.azimuthRad, 10.0, 1e-9);
}

TEST(RadarTrackFilter, TheF16ArrayGivesItsBeamwidths) {
    // 75.3 x 48.8 cm at 9.86 GHz, 1.2 lambda / D.
    double lambda = WavelengthM(9.86e9);
    EXPECT_NEAR(RadarTrackFilter::BeamwidthRad(lambda, 0.753, 1.2) * kRadToDeg, 2.78, 0.01);
    EXPECT_NEAR(RadarTrackFilter::BeamwidthRad(lambda, 0.488, 1.2) * kRadToDeg, 4.28, 0.01);
}

TEST(RadarTrackFilter, MeasurementAxesAreOrthonormalWithAzimuthLevel) {
    for (const Vec3& raw : {Vec3{1.0, 0.0, 0.0}, Vec3{0.3, -0.5, 0.8}, Vec3{0.0, 1.0, 0.0}}) {
        Vec3 los = raw.normalized();
        Vec3 az, el;
        RadarTrackFilter::MeasurementAxes(los, az, el);
        EXPECT_NEAR(az.norm(), 1.0, 1e-12);
        EXPECT_NEAR(el.norm(), 1.0, 1e-12);
        EXPECT_NEAR(az.dot(los), 0.0, 1e-12);
        EXPECT_NEAR(el.dot(los), 0.0, 1e-12);
        EXPECT_NEAR(az.dot(el), 0.0, 1e-12);
        if (std::fabs(los.y()) < 0.999) {
            EXPECT_NEAR(az.y(), 0.0, 1e-12);  // azimuth is horizontal
        }
    }
}

TEST(RadarTrackFilter, PredictCarriesTheTrackAndGrowsItsUncertainty) {
    TrackEstimate e;
    e.valid = true;
    e.state = {100.0, 200.0, 300.0, 10.0, -20.0, 30.0};
    for (int i = 0; i < 6; ++i) e.covariance[i * 6 + i] = 1.0;
    RadarTrackFilter::Predict(e, 2.0, 0.0);
    EXPECT_NEAR(e.state[0], 120.0, 1e-12);
    EXPECT_NEAR(e.state[1], 160.0, 1e-12);
    EXPECT_NEAR(e.state[2], 360.0, 1e-12);
    EXPECT_NEAR(e.covariance[0], 1.0 + 4.0, 1e-12);  // P_xx + dt^2 P_vv
    TrackEstimate noisy = e;
    RadarTrackFilter::Predict(noisy, 3.0, 10.0);
    RadarTrackFilter::Predict(e, 3.0, 0.0);
    EXPECT_GT(noisy.covariance[0], e.covariance[0]);  // maneuver allowance widens it
}

TEST(RadarTrackFilter, CovarianceMatchesTheActualErrorWhenTheModelIsTrue) {
    // A constant-velocity target and no maneuver allowance: the filter's
    // model is exact, so its mean NEES over many runs must be the state
    // dimension, 6 (chi-squared mean; sd of the mean sqrt(12/n)).
    RadarTrackFilter::LookSigmas sigmas{10.0, 2e-3, 3e-3};
    Rng rng;
    rng.Seed(42);
    constexpr int kRuns = 500;
    double neesSum = 0.0;
    for (int run = 0; run < kRuns; ++run) {
        Vec3 truePos, trueVel;
        TrackEstimate e = Track(rng, 30, 0.0, sigmas, truePos, trueVel);
        Eigen::Matrix<double, 6, 1> err;
        err << e.state[0] - truePos.x(), e.state[1] - truePos.y(), e.state[2] - truePos.z(), e.state[3] - trueVel.x(),
            e.state[4] - trueVel.y(), e.state[5] - trueVel.z();
        Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> p(e.covariance.data());
        neesSum += err.dot(p.inverse() * err);
    }
    EXPECT_NEAR(neesSum / kRuns, 6.0, 4.0 * std::sqrt(12.0 / kRuns));
}

TEST(RadarTrackFilter, SmoothsLooksIntoABetterEstimateThanAnyOne) {
    // With the 3 g allowance the radar uses, ten-odd looks still beat one:
    // the cross-range error ends well under a single look's.
    RadarTrackFilter::LookSigmas sigmas{10.0, 2e-3, 3e-3};
    Rng rng;
    rng.Seed(7);
    constexpr int kRuns = 400;
    double azSq = 0.0, velSq = 0.0;
    Vec3 truePos, trueVel;
    for (int run = 0; run < kRuns; ++run) {
        TrackEstimate e = Track(rng, 30, 3.0 * Units::kGravityFtPerSec2, sigmas, truePos, trueVel);
        Vec3 err = RadarTrackFilter::Position(e) - truePos;
        Vec3 los = truePos.normalized(), az, el;
        RadarTrackFilter::MeasurementAxes(los, az, el);
        azSq += err.dot(az) * err.dot(az);
        Vec3 velErr = RadarTrackFilter::Velocity(e) - trueVel;
        velSq += velErr.dot(velErr);
    }
    double oneLookAzFt = sigmas.azimuthRad * truePos.norm();
    EXPECT_LT(std::sqrt(azSq / kRuns), 0.75 * oneLookAzFt);
    // And it has found the target's velocity (250 m/s across the beam):
    // the error is a small fraction of it.
    EXPECT_LT(std::sqrt(velSq / kRuns), 0.25 * trueVel.norm());
}
