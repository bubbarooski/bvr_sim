#pragma once

#include "components/JSBSimComponent.h"
#include "components/KinematicsComponent.h"
#include "components/Velocity.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "util/Vec3.h"

// An entity's true velocity, from JSBSim or Kinematics. The
// *WithVelocityFallback overloads also accept a bare Velocity component
// (component-test scenarios with no flight dynamics); the plain ones return
// zero for such an entity.
//
// Store overloads exist so callers can hoist the store lookups out of
// per-pair sweep loops.
namespace EntityVelocity {

namespace detail {

// False (and `out` untouched) if neither JSBSim nor Kinematics has the entity.
// JSBSim NED -> world (x east, y up, z north).
inline bool TryMotionVelocity(ComponentStore<JSBSimComponent>& jsbsimComponents,
                               ComponentStore<KinematicsComponent>& kinematics, EntityID entity, Vec3& out) {
    if (const JSBSimComponent* jsbsim = jsbsimComponents.TryGet(entity)) {
        const JSBSimPropertyNodes& props = jsbsim->props;
        out = Vec3{props.vEastFps->getDoubleValue(), -props.vDownFps->getDoubleValue(),
                   props.vNorthFps->getDoubleValue()};
        return true;
    }
    if (const KinematicsComponent* kin = kinematics.TryGet(entity)) {
        out = kin->vel;
        return true;
    }
    return false;
}

}  // namespace detail

inline Vec3 Get(ComponentStore<JSBSimComponent>& jsbsimComponents, ComponentStore<KinematicsComponent>& kinematics,
                 EntityID entity) {
    Vec3 v{};
    detail::TryMotionVelocity(jsbsimComponents, kinematics, entity, v);
    return v;
}

inline Vec3 Get(World& world, EntityID entity) {
    return Get(world.Components<JSBSimComponent>(), world.Components<KinematicsComponent>(), entity);
}

inline Vec3 GetWithVelocityFallback(ComponentStore<JSBSimComponent>& jsbsimComponents,
                                     ComponentStore<KinematicsComponent>& kinematics,
                                     ComponentStore<Velocity>& velocities, EntityID entity) {
    Vec3 v{};
    if (detail::TryMotionVelocity(jsbsimComponents, kinematics, entity, v)) return v;
    if (const Velocity* vel = velocities.TryGet(entity)) return vel->linear;
    return Vec3{};
}

inline Vec3 GetWithVelocityFallback(World& world, EntityID entity) {
    return GetWithVelocityFallback(world.Components<JSBSimComponent>(), world.Components<KinematicsComponent>(),
                                    world.Components<Velocity>(), entity);
}

}  // namespace EntityVelocity
