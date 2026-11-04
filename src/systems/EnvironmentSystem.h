#pragma once

#include "ecs/World.h"
#include "sim/SimContext.h"

// Keeps every JSBSim aircraft's own atmosphere in step with the scenario's
// Environment. FlightDynamicsSystem::Init applies the conditions before
// trim; this applies the wind on the first tick and re-applies either if it
// changes mid-run (a live edit in the viewer), so JSBSim never flies
// different air from the rest of the sim. Draws nothing from the RNG.
namespace EnvironmentSystem {

void Update(World& world, SimContext& simContext);

}  // namespace EnvironmentSystem
