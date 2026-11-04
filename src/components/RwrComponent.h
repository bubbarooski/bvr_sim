#pragma once

#include "util/Numeric.h"

// What the RWR reads an emitter as doing. A scanning radar -- search or
// track-while-scan -- reads as SEARCH whatever it holds a track on; only a
// single-target-track lock on this aircraft reads as LOCK. See
// REFERENCE.md - Perceived emitter mode.
enum class RwrMode { NONE, SEARCH, LOCK };

// Radar warning receiver: a one-way link budget, far more sensitive than a
// radar's own round-trip detection (R^2 vs R^4 -- a physical asymmetry,
// not a tuning choice). See REFERENCE.md - Radar Warning Receiver.
// Its numbers are zero here, not a default receiver: it is built from its
// aircraft type (AircraftEntity::MakeRwr).
struct RwrComponent {
    double antennaGain = 0.0;     // Gr, linear
    double sensitivityDbm = 0.0;  // weakest pulse it registers, at the antenna port
    double signalJitterDb = 0.0;
    RwrMode perceivedMode = RwrMode::NONE;
    bool currentlyIlluminated = false;
    // A missile's own seeker is transmitting at this aircraft: the warning
    // that a missile is in its terminal phase, apart from whatever the
    // radars are doing. See REFERENCE.md - Missile warning.
    bool missileWarning = false;

    // Sweeps at its own rate (the type's), like every other stochastic
    // sensor here, so its draw rate on the shared RNG stream stays
    // proportionate.
    double sweepPeriodS = 0.0;
    double lastSweepTime = Numeric::kLongAgoS;
};
