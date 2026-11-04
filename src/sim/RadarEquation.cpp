#include "RadarEquation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "sim/AircraftBounds.h"
#include "util/Units.h"

namespace RadarEquation {

namespace {

using Units::kDegToRad;
using Units::kFeetPerMeter;
using Units::kPi;
using Units::kRadToDeg;

constexpr double kEarthRadiusM = 6371000.0;  // IUGG mean

constexpr double kMinRangeM = 1.0;         // keeps R^-4 and R^-2 finite at contact
constexpr double kMinPowerRatio = 1e-300;  // keeps log10 finite on a zero

// Radar sees further than the eye does: the atmosphere's vertical
// refractive-index gradient bends the beam downward, following the
// curvature slightly. Standard radar practice models this by inflating
// the Earth's radius by 4/3 rather than tracing the ray, which puts the
// horizon about 15% beyond the geometric one. See REFERENCE.md - Radar
// horizon.
constexpr double kEffectiveEarthRadiusM = 4.0 / 3.0 * kEarthRadiusM;

// (4*pi)^3, computed once (magic static, thread-safe): the radar systems
// call DetectorOutputDb for every in-view (radar, target) pair every sweep.
double FourPiCubed() {
    static const double value = std::pow(4.0 * kPi, 3);
    return value;
}

// k T0 F L: the receiver's noise power per hertz, with every loss folded in.
double NoiseDensityTimesLossW(const ReceiverBudget& budget) {
    return Units::kBoltzmannJPerK * Units::kReferenceNoiseTempK * std::pow(10.0, budget.noiseFigureDb / 10.0) *
           std::pow(10.0, budget.systemLossDb / 10.0);
}

double SnrLinear(const ReceiverBudget& budget, double sigmaM2, double rangeM) {
    double rangeSafeM = std::max(rangeM, kMinRangeM);
    return (budget.averagePowerW * budget.antennaGain * budget.antennaGain * budget.wavelengthM *
            budget.wavelengthM * sigmaM2 * budget.integrationTimeS) /
           (FourPiCubed() * std::pow(rangeSafeM, 4) * NoiseDensityTimesLossW(budget));
}

}  // namespace

double DetectionThresholdDb(double pfa) { return 10.0 * std::log10(std::log(1.0 / pfa)); }

double DetectorOutputDb(SimContext& simContext, const ReceiverBudget& budget, const RadarCrossSection& rcs,
                        double rangeM) {
    // Everything in units of the noise power: unit-power complex noise, so
    // the return's power is the mean SNR. Each quadrature of a unit-power
    // complex Gaussian has variance 1/2.
    const double quadratureSigma = std::sqrt(0.5);
    double meanSnr = SnrLinear(budget, rcs.staticRcsM2, rangeM);
    double amplitude = std::sqrt(meanSnr);
    double returnI = amplitude;
    double returnQ = 0.0;
    if (rcs.fluctuation == FluctuationModel::SWERLING_1) {
        // Many comparable scatterers: a complex Gaussian return, so its
        // power is exponential about the mean, fresh every look.
        returnI = amplitude * simContext.rng.NextGaussian(0.0, quadratureSigma);
        returnQ = amplitude * simContext.rng.NextGaussian(0.0, quadratureSigma);
    }
    double i = returnI + simContext.rng.NextGaussian(0.0, quadratureSigma);
    double q = returnQ + simContext.rng.NextGaussian(0.0, quadratureSigma);
    return 10.0 * std::log10(std::max(i * i + q * q, kMinPowerRatio));
}

double SwerlingOneDetectionProbability(double meanSnrDb, double thresholdDb) {
    return std::exp(-std::pow(10.0, thresholdDb / 10.0) / (1.0 + std::pow(10.0, meanSnrDb / 10.0)));
}

double SwerlingOneRequiredSnrDb(double thresholdDb, double pd) {
    // Inverting pd = exp(-T / (1 + S)) needs 0 < ln(1/pd) < T, or S is not a
    // positive number.
    const double threshold = std::pow(10.0, thresholdDb / 10.0);
    if (!(pd < 1.0) || !(std::log(1.0 / pd) < threshold)) {
        throw std::domain_error("SwerlingOneRequiredSnrDb: no positive SNR gives detection probability " +
                                std::to_string(pd) + " against a threshold of " + std::to_string(thresholdDb) + " dB");
    }
    return 10.0 * std::log10(std::pow(10.0, thresholdDb / 10.0) / std::log(1.0 / pd) - 1.0);
}

double SnrDb(const ReceiverBudget& budget, double rcsM2, double rangeM) {
    return 10.0 * std::log10(std::max(SnrLinear(budget, rcsM2, rangeM), kMinPowerRatio));
}

double ReferenceRangeM(const ReceiverBudget& budget, double rcsM2, double snrDb) {
    double snrAtOneMeter = SnrLinear(budget, rcsM2, 1.0);
    if (!(snrAtOneMeter > 0.0)) return 0.0;
    return std::pow(snrAtOneMeter / std::pow(10.0, snrDb / 10.0), 0.25);
}

double FriisReceivedPowerDb(double txPowerW, double txAntennaGain, double rxAntennaGain, double wavelengthM,
                            double rangeM) {
    double rangeSafeM = std::max(rangeM, kMinRangeM);
    return 10.0 * std::log10(txPowerW) + 10.0 * std::log10(txAntennaGain) + 10.0 * std::log10(rxAntennaGain) +
           20.0 * std::log10(wavelengthM) - 20.0 * std::log10(4.0 * kPi * rangeSafeM);
}

double HorizonRangeM(double radarAltitudeM, double targetAltitudeM) {
    return std::sqrt(2.0 * kEffectiveEarthRadiusM * std::max(radarAltitudeM, 0.0)) +
           std::sqrt(2.0 * kEffectiveEarthRadiusM * std::max(targetAltitudeM, 0.0));
}

double RangeM(const Vec3& a, const Vec3& b) {
    double dx = a.x() - b.x(), dy = a.y() - b.y(), dz = a.z() - b.z();
    return std::sqrt(dx * dx + dy * dy + dz * dz) / kFeetPerMeter;
}

Vec3 ForwardVector(const Vec3& eulerDeg) {
    double pitchRad = eulerDeg.y() * kDegToRad;
    double yawRad = eulerDeg.z() * kDegToRad;
    double cp = std::cos(pitchRad), sp = std::sin(pitchRad);
    double cy = std::cos(yawRad), sy = std::sin(yawRad);
    return Vec3{cp * sy, sp, cp * cy};
}

double ShapeProjectedAreaM2(const RcsEllipsoid* parts, int count, const Vec3& bodyUnit) {
    double areaFt2 = 0.0;
    for (int i = 0; i < count; ++i) {
        const RcsEllipsoid& e = parts[i];
        double across = e.heightFt * e.lengthFt * bodyUnit.x();  // seen from the side
        double above = e.spanFt * e.lengthFt * bodyUnit.y();     // seen from above or below
        double ahead = e.spanFt * e.heightFt * bodyUnit.z();     // seen nose- or tail-on
        areaFt2 += kPi * std::sqrt(across * across + above * above + ahead * ahead);
    }
    return areaFt2 / (kFeetPerMeter * kFeetPerMeter);
}

double MeanRcsM2(const RadarCrossSection& rcs, const Vec3& targetEulerDeg, const Vec3& targetToSensorUnit) {
    if (rcs.shapeCount == 0) return rcs.staticRcsM2 * rcs.rcsScale;
    const RcsEllipsoid* parts = rcs.shape.data();
    double noseOnM2 = ShapeProjectedAreaM2(parts, rcs.shapeCount, Vec3{0.0, 0.0, 1.0});
    Vec3 bodyUnit = AircraftBounds::WorldToBodyDirection(targetToSensorUnit, targetEulerDeg);
    return rcs.staticRcsM2 * ShapeProjectedAreaM2(parts, rcs.shapeCount, bodyUnit) / noseOnM2 * rcs.rcsScale;
}

bool InDopplerNotch(const Vec3& targetVelFtPerSec, const Vec3& losUnit, double notchThresholdMps) {
    double vRadialMps = std::fabs(targetVelFtPerSec.dot(losUnit)) / kFeetPerMeter;
    return vRadialMps < notchThresholdMps;
}

bool BeamOnGround(double radarAltitudeM, const Vec3& losUnit, double beamwidthRad) {
    double h = std::max(radarAltitudeM, 0.0);
    double horizonDipRad = std::acos(kEffectiveEarthRadiusM / (kEffectiveEarthRadiusM + h));
    double losElevationRad = std::asin(std::clamp(losUnit.y(), -1.0, 1.0));
    return losElevationRad - beamwidthRad < -horizonDipRad;
}

}  // namespace RadarEquation
