#pragma once

#include <optional>

#include "ecs/World.h"
#include "sim/SimContext.h"

// A selectable list of every live entity, plus what the selected one is
// doing: live telemetry and, in Advanced mode, the autopilot command (no
// schema key). Its scenario parameters live in the sidebar's Parameters
// section, which follows the same selection.
namespace EntityInspector {

void Draw(World& world, const SimContext& simContext, bool advancedMode);

// The selected entity, if any. Callers must still check it exists: the list
// drops stale IDs only when it next draws.
std::optional<EntityID> SelectedEntity();

// Selects `entity` as a list click would, so a 3D-view click and a list
// click behave the same.
void SetSelectedEntity(EntityID entity);

// Clears the selection, as clicking an already-selected entity does.
void ClearSelection();

}  // namespace EntityInspector
