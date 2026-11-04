#pragma once

#include "util/Vec3.h"

// Point-mass physical state, missile-only -- the missile is deliberately
// lighter-weight than an aircraft, with no six-DOF model. Position lives
// in Transform; this carries velocity and acceleration.
struct KinematicsComponent {
    Vec3 vel;
    double speed = 0.0;

    // Written by GuidanceSystem each tick, held constant over
    // IntegrationSystem's step.
    Vec3 accel;
};
