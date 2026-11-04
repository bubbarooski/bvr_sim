#pragma once

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"
#include "sim/SimContext.h"

// Resolves each active missile exactly once, at confirmed closest
// approach, by a probabilistic lethality roll against the interpolated
// miss distance -- or as a MISS if it exceeds max range without ever
// registering a pass. Records the outcome via simContext.eventLog. See
// REFERENCE.md - Engagement Resolution.
namespace InterceptSystem {

void Update(World& world, SimContext& simContext, const ScenarioData& scenario, double simTime);

// Sub-tick closest approach, exposed so it can be checked against a
// closed-form answer rather than only inferred from a whole engagement.
//
// Takes three consecutive per-tick range samples bracketing a minimum
// (rPrev being the smallest SAMPLED value) and returns the interpolated
// true minimum. tOut receives the vertex position in ticks relative to
// rPrev, clamped to [-1, 1].
double SubTickMinRangeFt(double rPrevPrev, double rPrev, double rCurrent, double& tOut);

// Pk-vs-miss-distance sigmoid, exposed for the same reason as
// SubTickMinRangeFt above: 1 at zero miss distance, falling through the
// kill radius toward 0. Unit-invariant as long as all three arguments
// agree; feet here, by the internal convention. See REFERENCE.md - 12.4
// Probabilistic lethality.
double Pk(double missDistanceFt, double killRadiusFt, double falloffWidthFt);

}  // namespace InterceptSystem
