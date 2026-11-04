#pragma once

#include <optional>

#include "ecs/World.h"

// What the camera-follow decision remembers between frames.
struct FollowSync {
    bool wasEnabled = false;
    std::optional<EntityID> syncedSelection;
};

// Once per frame. `enabled` is "Center Camera on Selection" while not placing
// waypoints. The camera starts following the selection when the selection
// changes or the option is switched on, and otherwise leaves `followed`
// alone, so a release (the F key, a drag, a double-click on empty space)
// holds until the next change -- following the selection every frame would
// take it straight back. Nothing selected means nothing followed.
inline void SyncFollow(bool enabled, std::optional<EntityID> selection, FollowSync& sync,
                       std::optional<EntityID>& followed) {
    if (enabled && (!sync.wasEnabled || selection != sync.syncedSelection)) {
        followed = selection;
        sync.syncedSelection = selection;
    }
    sync.wasEnabled = enabled;
}
