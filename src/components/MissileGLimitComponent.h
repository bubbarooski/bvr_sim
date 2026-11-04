#pragma once

#include <memory>

#include "missile/MissileNormalForce.h"

// The missile's own turn ceiling: the normal force at its largest angle of
// attack, capped structurally. Built from the missile's type
// (MissileEntity::MakeGLimit) for every real missile. Synthetic test
// missiles without one fall back to a flat structural clamp and fly
// without drag due to lift. See REFERENCE.md - Missile normal force and turn ceiling.
struct MissileGLimitComponent {
    std::shared_ptr<const MissileNormalForce> normalForce;  // the type's, shared; holds alpha_max
    double structuralGLimit = 0.0;
};
