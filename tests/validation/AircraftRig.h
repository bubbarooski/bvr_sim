#pragma once

#include <optional>

#include "FGFDMExec.h"

#include "TestRigs.h"

#include "aircraft/AircraftCatalog.h"
#include "components/AutopilotComponent.h"
#include "components/DestroyedComponent.h"
#include "components/JSBSimComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "scenario/ScenarioEntity.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"
#include "systems/ControlSystem.h"
#include "systems/EnvironmentSystem.h"
#include "systems/FlightDynamicsSystem.h"

namespace testrig {

// One trimmed JSBSim aircraft in its own world, with the autopilot holding
// its initial altitude, heading and speed. Step() advances the control
// loop, the environment and the flight model one fixed tick,
// in SimLoop's order.
struct AircraftRig {
    static constexpr double kDt = SimClock::kFixedDt;

    World world;
    SimContext ctx;
    EntityID id = 0;
    double t = 0.0;

    // `environment` is the scenario's air: its temperature reaches the
    // aircraft before trim, as the loader does it, its wind right after (as
    // the first tick's EnvironmentSystem does it in a full run), and ctx
    // carries it so every system reads the same air.
    // `aircraftType` is a type id under data/aircraft (testrig::kF16).
    AircraftRig(const char* aircraftType, double altitudeFt, double airspeedKts, double headingDeg,
                const Environment& environment = {}, std::optional<double> internalFuelFraction = {}) {
        ctx.environment = environment;
        id = world.CreateEntity();
        ScenarioEntity spawn;
        spawn.name = "TestAircraft";
        spawn.pos = Vec3{0.0, altitudeFt, 0.0};
        spawn.jsbsim.type = AircraftCatalog::Default().Get(aircraftType);
        spawn.jsbsim.altitudeFt = altitudeFt;
        spawn.jsbsim.headingDeg = headingDeg;
        spawn.jsbsim.airspeedKts = airspeedKts;
        spawn.jsbsim.temperatureOffsetC = environment.temperatureOffsetC;
        spawn.jsbsim.internalFuelFraction = internalFuelFraction;

        world.Components<Transform>().Add(id, Transform{spawn.pos, Vec3{0.0, 0.0, headingDeg}});
        FlightDynamicsSystem::Init(world, id, spawn);

        AutopilotComponent autopilot;
        autopilot.targetHeadingDeg = headingDeg;
        autopilot.targetAltitudeFt = altitudeFt;
        autopilot.targetSpeedKts = airspeedKts;
        world.Components<AutopilotComponent>().Add(id, autopilot);

        // Applies the spawn wind.
        EnvironmentSystem::Update(world, ctx);
    }

    JSBSim::FGFDMExec& Fdm() { return *world.Components<JSBSimComponent>().Get(id).fdm; }
    double Prop(const char* property) { return Fdm().GetPropertyValue(property); }
    AutopilotComponent& Autopilot() { return world.Components<AutopilotComponent>().Get(id); }
    const Transform& Pose() { return world.Components<Transform>().Get(id); }
    bool Destroyed() { return world.Components<DestroyedComponent>().Has(id); }

    void Step() {
        ControlSystem::Update(world, ctx, kDt, t);
        EnvironmentSystem::Update(world, ctx);
        FlightDynamicsSystem::Update(world, kDt);
        t += kDt;
    }
    void Fly(double seconds) {
        int ticks = static_cast<int>(seconds / kDt);
        for (int i = 0; i < ticks; ++i) Step();
    }
};

}  // namespace testrig
