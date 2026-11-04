#include "sim/Atmosphere.h"

#include <array>
#include <cmath>
#include <cstddef>

namespace Atmosphere {

namespace {

// Constants exactly as JSBSim's FGAtmosphere declares them, so the two
// atmospheres agree to rounding rather than to a tolerance.
constexpr double kFtToM = 0.3048;
constexpr double kRankinePerKelvin = 1.8;
constexpr double kGasConstant = 8.31432 / 0.0289645 / (kFtToM * kFtToM * kRankinePerKelvin);  // ft*lbf/(slug*degR)
constexpr double kGravity = 9.80665 / kFtToM;                                                   // ft/s^2
constexpr double kEarthRadiusFt = 6356766.0 / kFtToM;  // geopotential reference radius
constexpr double kSpecificHeatRatio = 1.4;
constexpr double kSeaLevelPressurePsf = 2116.228;

// Layer bases in GEOPOTENTIAL feet and their standard temperatures. Lapse
// rates are implied by consecutive rows rather than stored, so the table
// cannot contradict itself.
struct Layer {
    double baseGeopotentialFt;
    double baseTemperatureR;
};
constexpr std::array<Layer, 9> kLayers = {{
    {0.0, 518.67},
    {36089.2388, 389.97},   // tropopause, 11 km
    {65616.7979, 389.97},   // 20 km
    {104986.8766, 411.57},  // 32 km
    {154199.4751, 487.17},  // 47 km
    {167322.8346, 487.17},  // 51 km
    {232939.6325, 386.37},  // 71 km
    {278385.8268, 336.5028},  // 84.852 km geopotential, 86 km geometric
    {298556.4304, 336.5028},
}};
constexpr std::size_t kLayerCount = kLayers.size() - 1;  // the last row only closes the one before it

double LapseRate(std::size_t layer) {
    const Layer& base = kLayers[layer];
    const Layer& top = kLayers[layer + 1];
    return (top.baseTemperatureR - base.baseTemperatureR) / (top.baseGeopotentialFt - base.baseGeopotentialFt);
}

// Pressure across one layer, from its base pressure, under hydrostatic
// balance at the (offset) temperature profile.
double PressureInLayer(std::size_t layer, double basePressurePsf, double baseTemperatureR, double heightAboveBaseFt) {
    double lapse = LapseRate(layer);
    if (lapse != 0.0) {
        return basePressurePsf *
               std::pow(baseTemperatureR / (baseTemperatureR + lapse * heightAboveBaseFt), kGravity / (kGasConstant * lapse));
    }
    return basePressurePsf * std::exp(-kGravity * heightAboveBaseFt / (kGasConstant * baseTemperatureR));
}

// Pressure at each layer base for one temperature offset. Depends only on
// the offset, which is fixed per scenario, so it is computed once per
// offset rather than on every call -- the lookup runs several times per
// aircraft per tick. thread_local because batch workers run scenarios in
// parallel, each possibly at a different offset.
struct Breakpoints {
    double offsetR = 0.0;
    bool valid = false;
    std::array<double, kLayers.size()> pressurePsf{};
};

const Breakpoints& BreakpointsFor(double offsetR) {
    static thread_local Breakpoints cache;
    if (cache.valid && cache.offsetR == offsetR) return cache;
    cache.offsetR = offsetR;
    cache.pressurePsf[0] = kSeaLevelPressurePsf;
    for (std::size_t b = 0; b < kLayerCount; ++b) {
        double thickness = kLayers[b + 1].baseGeopotentialFt - kLayers[b].baseGeopotentialFt;
        cache.pressurePsf[b + 1] =
            PressureInLayer(b, cache.pressurePsf[b], kLayers[b].baseTemperatureR + offsetR, thickness);
    }
    cache.valid = true;
    return cache;
}

}  // namespace

AtmosphereState Lookup(double altitudeFt, double temperatureOffsetC) {
    double offsetR = temperatureOffsetC * kRankinePerKelvin;
    const Breakpoints& breakpoints = BreakpointsFor(offsetR);

    // The standard is defined in geopotential altitude, in which gravity
    // is constant with height; JSBSim converts the same way.
    double geopotentialFt = altitudeFt * kEarthRadiusFt / (kEarthRadiusFt + altitudeFt);

    // Below sea level the first layer's lapse rate simply continues.
    // Above the table's top the last layer continues the same way.
    std::size_t layer = 0;
    while (layer + 1 < kLayerCount && geopotentialFt >= kLayers[layer + 1].baseGeopotentialFt) ++layer;

    double heightAboveBaseFt = geopotentialFt - kLayers[layer].baseGeopotentialFt;
    double baseTemperatureR = kLayers[layer].baseTemperatureR + offsetR;
    double temperatureR = baseTemperatureR + LapseRate(layer) * heightAboveBaseFt;
    double pressurePsf = PressureInLayer(layer, breakpoints.pressurePsf[layer], baseTemperatureR, heightAboveBaseFt);

    return {pressurePsf / (kGasConstant * temperatureR), std::sqrt(kSpecificHeatRatio * kGasConstant * temperatureR),
            temperatureR, pressurePsf};
}

}  // namespace Atmosphere
