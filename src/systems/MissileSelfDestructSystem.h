#pragma once

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"
#include "sim/SimContext.h"

// A launched missile destroys itself once it can no longer do its job:
// after motor burnout, when the G it can still pull falls below its
// floor (too slow to steer), or when its time of flight runs out. Applies
// whether it is still guiding or coasting after a miss, so no missile
// flies on until it reaches the ground.
//
// Every self-destruct logs SELF_DESTRUCT (cause=min_g or max_tof). Still
// guiding, it also resolves the engagement as a miss: a MISS event with
// cause=self_destruct, impact region "self_destruct", miss distance the
// range to the target's airframe at that moment.
// Already coasting, it only removes the missile; the engagement was
// resolved by the miss. Runs after InterceptSystem, so a closest-approach
// pass on the same tick wins, and holds off while a guiding missile is
// within the kill radius or has a closest approach inside it awaiting
// confirmation, so the fuze, not the timer, scores that pass. See
// REFERENCE.md 12.7 - Self-destruct.
namespace MissileSelfDestructSystem {

void Update(World& world, SimContext& simContext, const ScenarioData& scenario, double simTime);

}  // namespace MissileSelfDestructSystem
