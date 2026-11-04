#pragma once

#include "ecs/World.h"
#include "sim/SimContext.h"

// An aircraft that reaches the ground plane is destroyed. Nothing else
// stops it: JSBSim flies a retracted-gear airframe straight through
// altitude zero, and an evading target that misjudges a low pull-out
// should pay for it.
//
// A target that hits the ground while a missile is in flight against it
// resolves that engagement as a kill credited to the shot -- the missile
// forced the maneuver that killed it -- recorded as an INTERCEPT with
// cause=ground_impact and impact region "ground_impact", so every
// consumer that counts kills counts it and analysis can still tell it
// apart. A crashed pursuer is simply removed.
//
// A missile that reaches the ground is destroyed too. One still guiding
// resolves its engagement as a MISS with cause=missile_ground_impact,
// Pk 0 and impact region "missile_ground_impact", as a self-destruct
// records its miss.
namespace GroundImpactSystem {

void Update(World& world, SimContext& simContext, double simTime);

}  // namespace GroundImpactSystem
