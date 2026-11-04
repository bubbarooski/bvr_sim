#include "sim/SimLoop.h"

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"
#include "sim/SimContext.h"
#include "systems/AirborneRadarSystem.h"
#include "systems/ControlSystem.h"
#include "systems/DatalinkSystem.h"
#include "systems/EnvironmentSystem.h"
#include "systems/FlightDynamicsSystem.h"
#include "systems/GroundImpactSystem.h"
#include "systems/GuidanceSystem.h"
#include "systems/IntegrationSystem.h"
#include "systems/InterceptSystem.h"
#include "systems/LaunchSystem.h"
#include "systems/MissileSelfDestructSystem.h"
#include "systems/RwrSystem.h"
#include "systems/WaypointSystem.h"

namespace SimLoop {

void BeginRun(ScenarioData& scenario, SimContext& simContext, uint64_t seed) {
    simContext.rng.Seed(seed);
    simContext.environment = scenario.environment;
}

void Step(World& world, SimContext& simContext, const ScenarioData& scenario, double dt, double& simTime) {
    simContext.environment = scenario.environment;
    WaypointSystem::Update(world, simContext.environment);
    ControlSystem::Update(world, simContext, dt, simTime);
    EnvironmentSystem::Update(world, simContext);
    FlightDynamicsSystem::Update(world, dt);
    GroundImpactSystem::Update(world, simContext, simTime);
    GuidanceSystem::Update(world, simContext, dt, simTime);
    IntegrationSystem::Update(world, dt);
    simTime += dt;
    AirborneRadarSystem::Update(world, simContext, simTime);
    DatalinkSystem::Update(world, simContext, dt, simTime);
    RwrSystem::Update(world, simContext, simTime);
    LaunchSystem::Update(world, simContext, scenario, simTime);
    InterceptSystem::Update(world, simContext, scenario, simTime);
    MissileSelfDestructSystem::Update(world, simContext, scenario, simTime);
}

}  // namespace SimLoop
