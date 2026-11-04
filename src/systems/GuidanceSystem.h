#pragma once

#include "components/GuidanceComponent.h"
#include "ecs/World.h"
#include "sim/SimContext.h"

// Proportional-navigation guidance for missile entities (GuidanceComponent
// + KinematicsComponent). Each tick it computes a lateral acceleration
// command from LOS rate and closing velocity, adds thrust and drag, and
// writes the sum into KinematicsComponent.accel for IntegrationSystem's
// RK4 step -- this system does not move the missile itself. Also runs the
// seeker gimbal cone, memory coast, and active-seeker acquisition. See
// REFERENCE.md - Proportional Navigation and Seeker Modeling.
namespace GuidanceSystem {

void Update(World& world, SimContext& simContext, double dt, double simTime);

// The missile's mass `timeSinceLaunch` into its flight: propellant burns
// off linearly over boost + sustain. See REFERENCE.md - Mass depletion.
double CurrentMassSlug(const GuidanceComponent& guidance, double timeSinceLaunch);

}  // namespace GuidanceSystem
