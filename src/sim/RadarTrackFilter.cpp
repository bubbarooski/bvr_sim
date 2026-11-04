#include "RadarTrackFilter.h"

#include <cmath>

#include <Eigen/Dense>

#include "util/Units.h"

namespace {

// Within this |cos| of vertical, "horizontal across the line of sight" is
// ill-conditioned.
constexpr double kNearVerticalCos = 0.999;

using Matrix6 = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
using Vector6 = Eigen::Matrix<double, 6, 1>;

Eigen::Map<Vector6> StateOf(TrackEstimate& e) { return Eigen::Map<Vector6>(e.state.data()); }
Eigen::Map<Matrix6> CovarianceOf(TrackEstimate& e) { return Eigen::Map<Matrix6>(e.covariance.data()); }

}  // namespace

namespace RadarTrackFilter {

double BeamwidthRad(double wavelengthM, double apertureM, double beamwidthFactor) {
    return beamwidthFactor * wavelengthM / apertureM;
}

LookSigmas MeasurementSigmas(double snrLinear, double azimuthBeamwidthRad, double elevationBeamwidthRad,
                             double monopulseSlope, double pulseWidthS) {
    double root = std::sqrt(2.0 * snrLinear);
    LookSigmas s;
    s.azimuthRad = azimuthBeamwidthRad / (monopulseSlope * root);
    s.elevationRad = elevationBeamwidthRad / (monopulseSlope * root);
    s.rangeM = Units::kSpeedOfLightMPerS * pulseWidthS / (2.0 * root);
    return s;
}

void MeasurementAxes(const Vec3& losUnit, Vec3& azimuthUnit, Vec3& elevationUnit) {
    // Straight up or down there is no horizontal across-axis; any one will do.
    Vec3 up = std::fabs(losUnit.y()) < kNearVerticalCos ? Vec3{0.0, 1.0, 0.0} : Vec3{1.0, 0.0, 0.0};
    azimuthUnit = up.cross(losUnit).normalized();
    elevationUnit = losUnit.cross(azimuthUnit);
}

void Predict(TrackEstimate& estimate, double time, double accelSigmaFtPerSec2) {
    if (!estimate.valid) return;
    double dt = time - estimate.time;
    if (dt <= 0.0) return;

    Matrix6 f = Matrix6::Identity();
    f.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * dt;

    // Discrete white-noise acceleration: one acceleration held over the
    // step, so position and velocity noise are correlated.
    double q = accelSigmaFtPerSec2 * accelSigmaFtPerSec2;
    Matrix6 noise = Matrix6::Zero();
    noise.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * (q * dt * dt * dt * dt / 4.0);
    noise.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * (q * dt * dt * dt / 2.0);
    noise.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * (q * dt * dt * dt / 2.0);
    noise.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * (q * dt * dt);

    StateOf(estimate) = f * StateOf(estimate);
    CovarianceOf(estimate) = f * CovarianceOf(estimate) * f.transpose() + noise;
    estimate.time = time;
}

void Update(TrackEstimate& estimate, double time, const Vec3& measuredFt, const Vec3& losUnit, double rangeFt,
            const LookSigmas& sigmas, double initialVelSigmaFtPerSec) {
    Vec3 azimuthUnit, elevationUnit;
    MeasurementAxes(losUnit, azimuthUnit, elevationUnit);
    Eigen::Matrix3d axes;
    axes.col(0) = Eigen::Vector3d(losUnit.x(), losUnit.y(), losUnit.z());
    axes.col(1) = Eigen::Vector3d(azimuthUnit.x(), azimuthUnit.y(), azimuthUnit.z());
    axes.col(2) = Eigen::Vector3d(elevationUnit.x(), elevationUnit.y(), elevationUnit.z());
    double rangeSigmaFt = sigmas.rangeM * Units::kFeetPerMeter;
    double azimuthSigmaFt = sigmas.azimuthRad * rangeFt;
    double elevationSigmaFt = sigmas.elevationRad * rangeFt;
    Eigen::Matrix3d lookCovariance = Eigen::Vector3d(rangeSigmaFt * rangeSigmaFt, azimuthSigmaFt * azimuthSigmaFt,
                                                     elevationSigmaFt * elevationSigmaFt)
                                         .asDiagonal();
    Eigen::Matrix3d r = axes * lookCovariance * axes.transpose();
    Eigen::Vector3d z(measuredFt.x(), measuredFt.y(), measuredFt.z());

    if (!estimate.valid) {
        estimate.valid = true;
        estimate.time = time;
        Vector6 x = Vector6::Zero();
        x.head<3>() = z;
        StateOf(estimate) = x;
        Matrix6 p = Matrix6::Zero();
        p.topLeftCorner<3, 3>() = r;
        p.bottomRightCorner<3, 3>() =
            Eigen::Matrix3d::Identity() * (initialVelSigmaFtPerSec * initialVelSigmaFtPerSec);
        CovarianceOf(estimate) = p;
        return;
    }

    // H = [I 0]: the radar measures position only.
    Matrix6 p = CovarianceOf(estimate);
    Eigen::Matrix3d innovationCovariance = p.topLeftCorner<3, 3>() + r;
    Eigen::Matrix<double, 6, 3> gain = p.leftCols<3>() * innovationCovariance.inverse();
    Vector6 x = StateOf(estimate);
    StateOf(estimate) = x + gain * (z - x.head<3>());
    Eigen::Matrix<double, 6, 6> ikh = Matrix6::Identity();
    ikh.leftCols<3>() -= gain;
    // Joseph form: stays symmetric positive definite under rounding.
    Matrix6 joseph = ikh * p * ikh.transpose();
    joseph += gain * r * gain.transpose();
    CovarianceOf(estimate) = joseph;
}

}  // namespace RadarTrackFilter
