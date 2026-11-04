#pragma once

#include <string>
#include <vector>

#include "components/AirborneRadarTrack.h"
#include "sim/Environment.h"
#include "sim/EventLog.h"
#include "sim/Rng.h"
#include "sim/TrackTable.h"
#include "util/Vec3.h"

// The engagement's milestones as typed fields, written by the systems
// that already compute them. The event log carries the same information,
// but only as formatted strings; this exists so a caller can branch on an
// outcome (and terminate a run the moment `resolved` goes true) without
// scanning and parsing the log.
//
// Times are seconds of sim time, negative meaning "never happened".
// Every "first" field latches on its first occurrence and is not
// overwritten by later ones.
struct EngagementOutcome {
    // LaunchSystem
    bool launched = false;
    double launchSimTime = -1.0;
    double launchRangeFt = 0.0;

    // AirborneRadarSystem: when a missile's launcher first detected that
    // missile's target; -1 until it does. Other aircraft's radars don't count.
    double firstDetectionSimTime = -1.0;
    double firingSolutionSimTime = -1.0;

    // GuidanceSystem
    double seekerActiveSimTime = -1.0;
    double seekerLostSimTime = -1.0;

    // How wrong the datalinked target position was when the seeker took
    // over -- the midcourse error, which sets where the seeker was looking
    // and so whether it acquired.
    double datalinkErrorAtHandoffFt = -1.0;
    double datalinkStalenessAtHandoffS = -1.0;

    // InterceptSystem. `resolved` means the engagement reached a terminal
    // state -- `hit` distinguishes which one.
    bool resolved = false;
    bool hit = false;
    bool armed = false;
    double missDistanceFt = 0.0;
    double pk = 0.0;
    std::string impactRegion;
    Vec3 impactBodyFt;
    double resolutionSimTime = -1.0;
};

// Owns the sim's single seeded RNG stream and the shared event log.
struct SimContext {
    Rng rng;
    EventLog eventLog;
    EngagementOutcome outcome;

    // The scenario's atmospheric conditions, mirrored from ScenarioData by
    // SimLoop::Step every tick so a live edit reaches every system at once.
    // A rig that never calls Step flies a standard day.
    Environment environment;

    // Radar tracks are keyed by a pair of entities, so they live here
    // rather than in a ComponentStore. An entry persists per pair for the
    // whole run: its presence does NOT mean a confirmed track -- check .mode.
    TrackTable<AirborneRadarTrack> airborneRadarTracks;
};
