#pragma once

#include <optional>
#include <string>

#include "components/Tag.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"

// The entity carrying a given Tag::name, or nullopt. EntityIDs are assigned
// fresh on every load, so lookup by name is the stable way to find one.
namespace EntityLookup {

inline std::optional<EntityID> FindByTagName(World& world, const std::string& name) {
    ComponentStore<Tag>& tags = world.Components<Tag>();
    for (auto& [entity, tag] : tags) {
        if (tag.name == name) return entity;
    }
    return std::nullopt;
}

}  // namespace EntityLookup
