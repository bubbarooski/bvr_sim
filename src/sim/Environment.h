#pragma once

#include "util/Vec3.h"

// Scenario-wide atmospheric conditions, parsed from the optional top-level
// "environment" block. Shared by everything that flies: JSBSim gets them at
// spawn (and on any later change, via EnvironmentSystem), and every
// Atmosphere lookup outside JSBSim reads them through SimContext, so the
// aircraft and the missile always fly in the same air.
struct Environment {
    // ISA deviation: every layer of the standard atmosphere runs this much
    // hotter (positive) or colder. See Atmosphere::Lookup.
    double temperatureOffsetC = 0.0;

    // One steady wind, the same at every altitude: its speed and the compass
    // direction it blows FROM (the meteorological convention, so 270 is a
    // westerly, moving air eastward).
    double windSpeedKts = 0.0;
    double windFromDeg = 0.0;
};

// The wind as the velocity of the air in the world frame (x east, y up,
// z north), ft/s. Exactly zero, not merely tiny, in calm air, so a calm
// scenario's arithmetic is untouched by subtracting it.
Vec3 WindFps(const Environment& environment);
