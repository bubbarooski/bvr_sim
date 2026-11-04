#pragma once

#include <memory>

#include "missile/MissileType.h"

// Which missile type an entity is. Attached by LaunchSystem::Init to every
// missile it creates. The type is shared and immutable: per-shot values
// (the scenario's overrides, guidance state) live in GuidanceComponent and
// MissileGLimitComponent.
struct MissileComponent {
    std::shared_ptr<const MissileType> type;
};
