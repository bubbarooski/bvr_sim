#pragma once

#include "util/Vec3.h"

// World convention: X=east, Y=up, Z=north. XZ is the horizontal ground
// plane and pos.y() is altitude. Scenario JSON pos arrays are [x, y, z]:
// altitude in the wrong slot parses fine and silently places the entity on
// or under the ground.
struct Transform {
    Vec3 pos;
    Vec3 euler;
};
