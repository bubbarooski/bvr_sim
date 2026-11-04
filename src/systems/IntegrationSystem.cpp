#include "IntegrationSystem.h"

#include <cmath>

#include "components/DestroyedComponent.h"
#include "components/KinematicsComponent.h"
#include "components/Transform.h"
#include "components/Velocity.h"
#include "ecs/ComponentStore.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace {

using Units::kRadToDeg;

}  // namespace

namespace IntegrationSystem {

void Update(World& world, double dt) {
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    ComponentStore<Velocity>& velocities = world.Components<Velocity>();

    for (auto& [entity, velocity] : velocities) {
        Transform* transform = transforms.TryGet(entity);
        if (transform == nullptr) continue;

        transform->pos.x() += velocity.linear.x() * dt;
        transform->pos.y() += velocity.linear.y() * dt;
        transform->pos.z() += velocity.linear.z() * dt;
    }

    // Missiles advance by a constant-acceleration step: GuidanceSystem's
    // acceleration is held fixed across the tick, so position gets the
    // exact v*dt + a*dt^2/2. It is written in RK4 stage form, which with a
    // constant forcing term collapses to that same result; the stage form
    // is kept only because rewriting it would change rounding and so every
    // recorded trajectory. It is not RK4 over the guidance dynamics -- the
    // acceleration is not re-evaluated at the stages.
    //
    // Only a destroyed missile freezes. A missed one keeps integrating
    // under gravity and drag until the ground check below stops it.
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    ComponentStore<KinematicsComponent>& kinematics = world.Components<KinematicsComponent>();
    for (auto& [entity, kin] : kinematics) {
        if (destroyed.Has(entity)) continue;
        Transform* transformPtr = transforms.TryGet(entity);
        if (transformPtr == nullptr) continue;
        Transform& transform = *transformPtr;
        double prevY = transform.pos.y();

        Vec3 vel = kin.vel;
        Vec3 accel = kin.accel;

        Vec3 k1Pos = vel;
        Vec3 k2Pos = vel + accel * (dt / 2.0);
        Vec3 k3Pos = vel + accel * (dt / 2.0);
        Vec3 k4Pos = vel + accel * dt;

        Vec3 posSum = k1Pos + k2Pos * 2.0 + (k3Pos * 2.0 + k4Pos);
        transform.pos += posSum * (dt / 6.0);

        kin.vel = vel + accel * dt;
        kin.speed = kin.vel.norm();

        // Requires a genuine downward crossing this tick, not merely
        // "at or below 0", so a synthetic test missile starting at the
        // origin is not frozen on its first tick.
        if (prevY > 0.0 && transform.pos.y() <= 0.0) {
            transform.pos.y() = 0.0;
            destroyed.Add(entity, DestroyedComponent{});
        }

        // A fin-stabilized missile weathervanes, so attitude is just the
        // velocity direction. Roll stays 0 -- a point mass has no bank
        // model, and yaw+pitch already determine where it points.
        double horizontalSpeed = std::sqrt(kin.vel.x() * kin.vel.x() + kin.vel.z() * kin.vel.z());
        double yawDeg = std::atan2(kin.vel.x(), kin.vel.z()) * kRadToDeg;
        double pitchDeg = std::atan2(kin.vel.y(), horizontalSpeed) * kRadToDeg;
        transform.euler = Vec3{0.0, pitchDeg, yawDeg};
    }
}

}  // namespace IntegrationSystem
