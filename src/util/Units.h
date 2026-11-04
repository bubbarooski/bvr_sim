#pragma once

// Shared unit-conversion and physical constants.
namespace Units {

constexpr double kFeetPerMeter = 3.280839895;
constexpr double kFeetSqPerMeterSq = kFeetPerMeter * kFeetPerMeter;
constexpr double kLbfPerNewton = 0.2248089430997105;
constexpr double kKgPerSlug = 14.5939029372;  // one slug is this many kg: divide kg by it to get slugs
constexpr double kGravityFtPerSec2 = 32.174;
constexpr double kFpsPerKt = 1.6878098571;  // international knot, 1852 m/h
constexpr double kMetersPerNmi = 1852.0;
constexpr double kFeetPerNmi = kMetersPerNmi * kFeetPerMeter;

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMradToRad = 0.001;

constexpr double kSpeedOfLightMPerS = 299792458.0;  // exact, SI definition
constexpr double kBoltzmannJPerK = 1.380649e-23;    // exact, SI definition
// The IEEE reference temperature that noise figures are quoted against.
constexpr double kReferenceNoiseTempK = 290.0;

constexpr double kDbmPerDbw = 30.0;  // 1 W = 30 dBm: dBW = dBm - this

}  // namespace Units
