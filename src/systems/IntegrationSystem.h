#pragma once

#include "ecs/World.h"

// Advances missile state by a constant-acceleration step from
// GuidanceSystem's acceleration, and any plain Velocity entity by Euler.
namespace IntegrationSystem {

void Update(World& world, double dt);

}  // namespace IntegrationSystem
