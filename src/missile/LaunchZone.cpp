#include "missile/LaunchZone.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <tuple>
#include <utility>

#include "sim/Atmosphere.h"
#include "util/TypeFileJson.h"
#include "util/Units.h"

namespace {

// 64-bit FNV-1a (Fowler-Noll-Vo) parameters.
constexpr uint64_t kFnvOffsetBasis = 14695981039346656037ull;
constexpr uint64_t kFnvPrime = 1099511628211ull;

// Angle wrapped into (-180, 180].
double WrapDeg(double deg) {
    double wrapped = std::fmod(deg + 180.0, 360.0);
    if (wrapped <= 0.0) wrapped += 360.0;
    return wrapped - 180.0;
}

// Heading of a horizontal vector, degrees from +Z toward +X -- the
// convention IntegrationSystem uses for yaw.
double HeadingDeg(const Vec3& v) { return std::atan2(v.x(), v.z()) * Units::kRadToDeg; }

// The cell below `value` on `axis` and the fraction across it, clamped to
// the axis's ends.
std::pair<size_t, double> Bracket(const std::vector<double>& axis, double value) {
    if (value <= axis.front()) return {0, 0.0};
    if (value >= axis.back()) return {axis.size() - 2, 1.0};
    size_t upper = static_cast<size_t>(std::upper_bound(axis.begin(), axis.end(), value) - axis.begin());
    size_t lower = upper - 1;
    return {lower, (value - axis[lower]) / (axis[upper] - axis[lower])};
}

}  // namespace

LaunchZone::LaunchZone(std::array<std::vector<double>, kAxes> axes, std::vector<double> rmaxNmi,
                       std::string missileFileHash)
    : axes_(std::move(axes)), rmaxNmi_(std::move(rmaxNmi)), missileFileHash_(std::move(missileFileHash)) {}

LaunchZone LaunchZone::Load(const std::filesystem::path& file) {
    const std::string where = file.string();
    const nlohmann::json j = TypeFileJson::Read(file, "LaunchZone");
    std::string hash = TypeFileJson::RequireString(j, "missile_file_hash", where);

    const nlohmann::json& axesJson = TypeFileJson::RequireObject(j, "axes", where);
    std::array<std::vector<double>, kAxes> axes;
    size_t cells = 1;
    for (int a = 0; a < kAxes; ++a) {
        const char* name = kAxisNames[a];
        if (!axesJson.contains(name) || !axesJson.at(name).is_array() || axesJson.at(name).size() < 2) {
            throw std::runtime_error(where + ": axes.\"" + name + "\" must be an array of at least two numbers");
        }
        for (const nlohmann::json& value : axesJson.at(name)) {
            if (!value.is_number() || (!axes[a].empty() && value.get<double>() <= axes[a].back())) {
                throw std::runtime_error(where + ": axes.\"" + name + "\" must be strictly increasing numbers");
            }
            axes[a].push_back(value.get<double>());
        }
        cells *= axes[a].size();
    }
    // The geometry folds every launch into these ranges.
    const std::vector<double>& aspect = axes[kAspectAxis];
    const std::vector<double>& offBoresight = axes[kOffBoresightAxis];
    if (aspect.front() < -180.0 || aspect.back() > 180.0 || offBoresight.front() < 0.0 ||
        offBoresight.back() > 180.0) {
        throw std::runtime_error(where + ": aspect_deg must lie in [-180, 180] and off_boresight_deg in [0, 180]");
    }

    if (!j.contains("rmax_nmi") || !j.at("rmax_nmi").is_array() || j.at("rmax_nmi").size() != cells) {
        throw std::runtime_error(where + ": \"rmax_nmi\" must hold one value per grid cell (" + std::to_string(cells) +
                                 ")");
    }
    std::vector<double> rmax;
    rmax.reserve(cells);
    for (const nlohmann::json& value : j.at("rmax_nmi")) {
        if (!value.is_number() || value.get<double>() < 0.0) {
            throw std::runtime_error(where + ": every \"rmax_nmi\" value must be a number, not negative");
        }
        rmax.push_back(value.get<double>());
    }
    return LaunchZone(std::move(axes), std::move(rmax), std::move(hash));
}

LaunchGeometry LaunchZone::Geometry(const Vec3& shooterPos, const Vec3& shooterVel, const Vec3& targetPos,
                                    const Vec3& targetVel, double temperatureOffsetC) {
    LaunchGeometry g;
    g.shooterAltitudeFt = shooterPos.y();
    double speedOfSound = Atmosphere::Lookup(shooterPos.y(), temperatureOffsetC).speedOfSoundFtPerSec;
    g.shooterMach = shooterVel.norm() / speedOfSound;
    g.targetMach = targetVel.norm() / speedOfSound;
    Vec3 los = targetPos - shooterPos;
    g.rangeFt = los.norm();

    double bearingDeg = HeadingDeg(los);
    double offBoresight = WrapDeg(bearingDeg - HeadingDeg(shooterVel));
    // From the target's tail (its heading + 180) round to the shooter (the
    // bearing + 180): the two 180s cancel.
    double aspect = WrapDeg(bearingDeg - HeadingDeg(targetVel));
    if (offBoresight < 0.0) {
        offBoresight = -offBoresight;
        aspect = aspect == 180.0 ? 180.0 : -aspect;
    }
    g.offBoresightDeg = offBoresight;
    g.aspectDeg = aspect;
    return g;
}

double LaunchZone::RmaxFt(const LaunchGeometry& geometry) const {
    const std::array<double, kAxes> point = {geometry.shooterAltitudeFt, geometry.shooterMach, geometry.targetMach,
                                             geometry.aspectDeg, geometry.offBoresightDeg};
    std::array<size_t, kAxes> lower{};
    std::array<double, kAxes> fraction{};
    std::array<size_t, kAxes> stride{};
    size_t s = 1;
    for (int a = kAxes - 1; a >= 0; --a) {
        std::tie(lower[a], fraction[a]) = Bracket(axes_[a], point[a]);
        stride[a] = s;
        s *= axes_[a].size();
    }
    // Sum over the 2^5 corners of the cell, each weighted by the product
    // of its per-axis fractions.
    double rmax = 0.0;
    for (int corner = 0; corner < (1 << kAxes); ++corner) {
        double weight = 1.0;
        size_t index = 0;
        for (int a = 0; a < kAxes; ++a) {
            bool upper = (corner >> a) & 1;
            weight *= upper ? fraction[a] : 1.0 - fraction[a];
            index += (lower[a] + (upper ? 1 : 0)) * stride[a];
        }
        if (weight != 0.0) rmax += weight * rmaxNmi_[index];
    }
    return rmax * Units::kFeetPerNmi;
}

std::string LaunchZone::FileHash(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    if (!in) throw std::runtime_error("LaunchZone: cannot read " + file.string());
    uint64_t hash = kFnvOffsetBasis;
    for (std::istreambuf_iterator<char> it(in), end; it != end; ++it) {
        if (*it == '\r') continue;
        hash ^= static_cast<unsigned char>(*it);
        hash *= kFnvPrime;
    }
    char hex[17];
    std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(hash));
    return hex;
}
