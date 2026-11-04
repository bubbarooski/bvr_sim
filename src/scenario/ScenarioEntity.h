#pragma once

#include <memory>
#include <optional>
#include <string>

#include "aircraft/AircraftType.h"

#include "util/Vec3.h"

// An aircraft's type and initial conditions, parsed from
// entities[].components.jsbsim in the scenario JSON. Only present when that
// key exists.
struct ScenarioJSBSimData {
    // Resolved from jsbsim.aircraft through the AircraftCatalog; never null
    // once loaded.
    std::shared_ptr<const AircraftType> type;
    double altitudeFt = 0.0;
    double headingDeg = 0.0;
    double airspeedKts = 0.0;

    // Fraction (0-1) of the type's internal tank capacity filled at spawn,
    // for test rigs matching a published weight; the loader never sets it.
    // Absent keeps the load its flight model file declares. External tanks
    // stay empty: aircraft fly clean, with no drop-tank drag modelled.
    std::optional<double> internalFuelFraction;

    // Scenario-wide, copied in by the loader: JSBSim must be flying the
    // scenario's air before it trims, or the trim solves for the wrong one.
    double temperatureOffsetC = 0.0;
};

// Parsed scenario data for one entity, for systems needing more than the
// plain Transform/Tag the loader already attaches. Only fields something
// actually reads live here -- the JSON's "type" key and outer transform
// heading stay schema-required but are not stored.
struct ScenarioEntity {
    std::string name;
    Vec3 pos;
    ScenarioJSBSimData jsbsim;
};
