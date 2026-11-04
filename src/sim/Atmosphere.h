#pragma once

// U.S. Standard Atmosphere 1976, evaluated in closed form: density, speed
// of sound, temperature and pressure vs. altitude, in the English units
// (feet, slug/ft^3, ft/s, degR, lbf/ft^2) the rest of this sim's flight
// physics already uses. See REFERENCE.md - Atmosphere.
//
// Built to agree with JSBSim's own FGStandardAtmosphere, which flies every
// aircraft, so the missile, the radar horizon and the aircraft all fly in
// the same air: the same layer table, the same gas constants, the same
// geometric-to-geopotential conversion, and the same treatment of a
// temperature offset.
namespace Atmosphere {

struct AtmosphereState {
    double densitySlugPerFt3;
    double speedOfSoundFtPerSec;
    double temperatureR;
    double pressurePsf;
};

// `altitudeFt` is geometric altitude above sea level. Valid from below sea
// level to about 282 000 ft (86 km geometric, 84.852 km geopotential), the
// top of the 1976 standard's lower atmosphere; above that the top layer is
// extended isothermally.
//
// `temperatureOffsetC` is an ISA deviation: every layer runs that much
// hotter or colder, sea-level pressure stays standard, and pressure aloft
// follows hydrostatically from the offset temperatures -- so a hot day is
// thinner at altitude, not just warmer. Same convention as JSBSim's
// SetTemperatureBias.
AtmosphereState Lookup(double altitudeFt, double temperatureOffsetC = 0.0);

}  // namespace Atmosphere
