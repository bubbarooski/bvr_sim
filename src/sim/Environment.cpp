#include "sim/Environment.h"

#include <cmath>

#include "util/Units.h"

Vec3 WindFps(const Environment& environment) {
    if (environment.windSpeedKts == 0.0) return Vec3{};
    double speedFps = environment.windSpeedKts * Units::kFpsPerKt;
    double fromRad = environment.windFromDeg * Units::kDegToRad;
    return Vec3{-speedFps * std::sin(fromRad), 0.0, -speedFps * std::cos(fromRad)};
}
