#pragma once

#include "ecs/World.h"
#include "sim/SimContext.h"

// Cascaded autopilot: for every entity with both an AutopilotComponent and
// a JSBSimComponent, drives aileron/elevator/throttle toward the
// autopilot's heading/altitude/speed targets. Also enforces
// GLimitComponent's V-n envelope and accumulates structural damage;
// entities without that component are unaffected. See REFERENCE.md - The
// cascaded autopilot and The Flight Envelope.
namespace ControlSystem {

void Update(World& world, SimContext& simContext, double dt, double simTime);

}  // namespace ControlSystem
