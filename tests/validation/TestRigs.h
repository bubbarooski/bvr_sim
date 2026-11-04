#pragma once

// Setup shared by the validation tests. Deliberately setup only: the
// physics each test checks is still re-derived independently in the test
// itself, never through a helper here. JSBSim-free; the aircraft rig lives
// in AircraftRig.h.

#include <memory>

#include "aircraft/AircraftCatalog.h"
#include "components/AircraftComponent.h"
#include "components/GuidanceComponent.h"
#include "components/KinematicsComponent.h"
#include "components/MissileComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileCatalog.h"
#include "missile/MissileEntity.h"
#include "util/Vec3.h"

namespace testrig {

// The aircraft type the validation tests fly and shoot at.
constexpr const char* kF16 = "f16c_block50";
inline std::shared_ptr<const AircraftType> F16() { return AircraftCatalog::Default().Get(kF16); }

// Makes a bare test entity an F-16 for anything that asks its type -- the
// missile's miss distance is measured against the target type's airframe.
inline void GiveF16Airframe(World& world, EntityID entity) {
    world.Components<AircraftComponent>().Add(entity, AircraftComponent{F16()});
}

// The missile type the validation tests fly.
constexpr const char* kAim120 = "aim120";
inline std::shared_ptr<const MissileType> Aim120() { return MissileCatalog::Default().Get(kAim120); }

// A synthetic test missile's guidance: the AIM-120's motor, mass, drag
// area, seeker hardware and fuze, but an autonomous ARH seeker (it has no
// launcher to be illuminated by), no response lag, no Doppler notch (many
// rigs aim at a stationary target, which a pulse-Doppler seeker cannot see)
// and its launch turn already behind it (the rigs start mid-flight), so a
// test sees exactly the one mechanism it checks.
inline GuidanceComponent SyntheticAim120Guidance() {
    GuidanceComponent guidance = MissileEntity::MakeGuidance(*Aim120());
    guidance.seekerMode = SeekerMode::ARH;
    guidance.responseTimeS = 0.0;
    guidance.seekerNotchVelocityThresholdMps = 0.0;
    guidance.launchTurnDone = true;
    return guidance;
}

// Makes a bare test entity an AIM-120 for anything that asks its type (its
// drag table), and returns its synthetic guidance for the caller to adjust
// and attach.
inline GuidanceComponent GiveAim120(World& world, EntityID missile) {
    world.Components<MissileComponent>().Add(missile, MissileComponent{Aim120()});
    return SyntheticAim120Guidance();
}

// An entity moving as a point mass: Transform plus KinematicsComponent.
inline EntityID SpawnMover(World& world, const Vec3& pos, const Vec3& vel) {
    EntityID entity = world.CreateEntity();
    world.Components<Transform>().Add(entity, Transform{pos, Vec3{}});
    KinematicsComponent kinematics;
    kinematics.vel = vel;
    kinematics.speed = vel.norm();
    world.Components<KinematicsComponent>().Add(entity, kinematics);
    return entity;
}

}  // namespace testrig
