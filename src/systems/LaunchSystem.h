#pragma once

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"
#include "sim/SimContext.h"

// Fires each pre-created missile from its launcher, at the launcher's
// current position and velocity, once the launcher's airborne radar holds a
// confirmed track on the missile's target inside the launch zone (and, for
// an STT shot, a single-target-track lock) for the fire-control delay. One
// missile per engagement. Records a LAUNCH event. Also fire control's side
// of the lock: it locks and releases the launcher radar's STT
// (RADAR_LOCK / RADAR_LOCK_RELEASED).
namespace LaunchSystem {

// Pre-creates the "Missile" entity so it is inspectable before it fires:
// a Tag, its MissileComponent (the scenario's missile type), an unlaunched
// GuidanceComponent naming `pursuer` as its launcher and `target` as its
// target, a MissileGLimitComponent, and a DatalinkTrackComponent (the
// launcher's relayed target track). No Transform or Kinematics yet, so
// it does not render or leave a trail. Update populates the same entity at
// actual launch. `scenario` must have a missile type.
void Init(World& world, EntityID pursuer, EntityID target, const ScenarioData& scenario);

// Copies the scenario's doctrine into every missile not yet launched, so a
// "(next launch)" edit made before the first launch reaches the missile
// Init pre-created at load. Launched missiles keep what they fired with.
void RefreshUnlaunched(World& world, const ScenarioData& scenario);

// Init for a scenario's "Pursuer"- and "Target"-tagged entities, which is
// how every compound scenario names its engagement. Returns false, doing
// nothing, if the scenario lacks either or has no missile -- a component
// scenario with no engagement.
bool InitFromTags(World& world, const ScenarioData& scenario);

void Update(World& world, SimContext& simContext, const ScenarioData& scenario, double simTime);

}  // namespace LaunchSystem
