#pragma once

#include "ecs/World.h"
#include "sim/SimContext.h"

// Radar warning receiver. For every entity with an RwrComponent, evaluates
// every AirborneRadarComponent in the scene as a potential illuminator via
// the Friis one-way link (R^2 falloff, not the round-trip R^4), reads
// search or lock, and warns when a missile's own seeker is transmitting at
// it. See REFERENCE.md - Radar Warning Receiver.
namespace RwrSystem {

void Update(World& world, SimContext& simContext, double simTime);

}  // namespace RwrSystem
