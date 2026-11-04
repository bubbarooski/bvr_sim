#pragma once

#include <array>

#include "util/Vec3.h"

// What a radar's tracker believes about one target: a constant-velocity
// Kalman filter over world position and velocity (ft, ft/s), fed one
// noisy measurement per detection. See REFERENCE.md - Track measurement
// and filtering.
struct TrackEstimate {
    bool valid = false;  // false until the first detection, and after the track is dropped
    double time = 0.0;   // sim time the state refers to
    std::array<double, 6> state{};       // x, y, z, vx, vy, vz
    std::array<double, 36> covariance{};  // row-major 6x6
};

namespace RadarTrackFilter {

// 1-sigma errors of one look, in the radar's measurement axes.
struct LookSigmas {
    double rangeM = 0.0;
    double azimuthRad = 0.0;
    double elevationRad = 0.0;
};

// The radar's half-power beamwidth across an aperture `apertureM` wide:
// beamwidthFactor * lambda / D radians (~1.2 for a tapered aperture).
double BeamwidthRad(double wavelengthM, double apertureM, double beamwidthFactor);

// One look's measurement errors at signal-to-noise ratio snrLinear (MIT
// Lincoln Laboratory, Parameter Estimation and Tracking: sigma ~
// resolution / sqrt(SNR)): monopulse angle theta3 / (km sqrt(2 SNR)) per
// axis, range c tau / (2 sqrt(2 SNR)).
LookSigmas MeasurementSigmas(double snrLinear, double azimuthBeamwidthRad, double elevationBeamwidthRad,
                             double monopulseSlope, double pulseWidthS);

// The radar's measurement axes for a line of sight losUnit: azimuth
// (horizontal, across the line of sight) and elevation (up, across it).
void MeasurementAxes(const Vec3& losUnit, Vec3& azimuthUnit, Vec3& elevationUnit);

// Advances the estimate to `time` under a constant-velocity model with
// white acceleration noise of 1-sigma accelSigmaFtPerSec2 per axis. A
// no-op on an invalid estimate.
void Predict(TrackEstimate& estimate, double time, double accelSigmaFtPerSec2);

// Folds in a measured position (ft). The measurement's covariance is
// diagonal in the look's own axes -- range along losUnit, azimuth and
// elevation across it, the angular sigmas scaled by rangeFt -- and
// rotated into world axes. An invalid estimate starts at the measurement
// with velocity unknown (1-sigma initialVelSigmaFtPerSec per axis).
void Update(TrackEstimate& estimate, double time, const Vec3& measuredFt, const Vec3& losUnit, double rangeFt,
            const LookSigmas& sigmas, double initialVelSigmaFtPerSec);

inline Vec3 Position(const TrackEstimate& e) { return Vec3{e.state[0], e.state[1], e.state[2]}; }
inline Vec3 Velocity(const TrackEstimate& e) { return Vec3{e.state[3], e.state[4], e.state[5]}; }

}  // namespace RadarTrackFilter
