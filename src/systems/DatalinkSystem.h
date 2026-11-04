#pragma once

#include "ecs/World.h"
#include "sim/SimContext.h"

// Maintains each in-flight missile's belief about where its target is,
// as relayed by the launch aircraft's radar over the datalink.
//
// Runs after the radar systems, so an update carries this tick's track
// state rather than last tick's. Guidance reads the result on the
// following tick, which is a real (if small) datalink latency.
namespace DatalinkSystem {

void Update(World& world, SimContext& simContext, double dt, double simTime);

}  // namespace DatalinkSystem
