#pragma once

#include "util/Vec3.h"

// Where an entity starts, as authored (plus any pre-run edit). Transform is
// the live pose and is overwritten every tick once the run starts, so this
// is what setup-timed parameters edit and what Save/Reset write back --
// capturing Transform mid-flight would move the start line to wherever
// the entity happened to be.
struct SpawnPose {
    Vec3 pos;
    double headingDeg = 0.0;
};
