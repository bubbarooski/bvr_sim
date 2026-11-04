#include "missile/MissileNormalForce.h"

#include <algorithm>
#include <cmath>

#include "sim/TableLookup.h"
#include "util/Units.h"

namespace {

double Interpolate(const MissileNormalForce::Table& table, double mach) {
    using Row = MissileNormalForce::Table::value_type;
    return TableLookup::Lookup(table, mach, &Row::first, &Row::second);
}

}  // namespace

std::pair<double, double> MissileNormalForce::Coefficients(double mach) const {
    double a = Interpolate(cnAlphaPerDeg_, mach);
    double b = (Interpolate(cnMax_, mach) - a * alphaMaxDeg_) / (alphaMaxDeg_ * alphaMaxDeg_);
    return {a, b};
}

double MissileNormalForce::Cn(double alphaRad, double mach) const {
    double alphaDeg = std::min(std::fabs(alphaRad) * Units::kRadToDeg, alphaMaxDeg_);
    auto [a, b] = Coefficients(mach);
    return a * alphaDeg + b * alphaDeg * alphaDeg;
}

// b alpha^2 + a alpha - cn = 0, the positive root; b = 0 is the straight
// line. Written as 2 cn / (a + sqrt(a^2 + 4 b cn)) so a small b loses no
// precision.
double MissileNormalForce::AlphaForCn(double cn, double mach) const {
    if (cn <= 0.0) return 0.0;
    auto [a, b] = Coefficients(mach);
    double alphaDeg = 2.0 * cn / (a + std::sqrt(a * a + 4.0 * b * cn));
    return std::min(alphaDeg, alphaMaxDeg_) * Units::kDegToRad;
}

double MissileNormalForce::CnMax(double mach) const { return Interpolate(cnMax_, mach); }

double MissileNormalForce::AlphaMaxRad() const { return alphaMaxDeg_ * Units::kDegToRad; }
