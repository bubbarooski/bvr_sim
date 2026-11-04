#pragma once

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"

// Renders ParamRegistry entries as editable widgets, so the viewer's
// editor is generated from the same table the saver writes and the batch
// runner overrides. Adding a schema field makes it appear here with no
// UI code at all.
namespace ParameterPanel {

// Every scenario parameter belonging to one entity, grouped by subsystem:
// for a missile, the missile_doctrine block (stored scenario-wide, but it
// describes the missile); for an aircraft, its placement, envelope,
// signature, radar and RWR, and the environment it flies in.
//
// Outside Advanced mode only the basic knobs show: no type values, no
// advanced knobs (ParamRegistry::ParamDef::advanced).
//
// Returns false if nothing was drawn, so the caller can say so rather than
// showing an empty panel.
bool DrawForEntity(ScenarioData& scenario, World& world, EntityID entity, double simTime, bool advancedMode);

// True (once) if a "(setup)" placement edit was committed since the last
// call -- a drag released or a typed value entered. The flight model was
// started from the old pose at load, so the caller must re-apply the start
// with a keep-edits Reset, or the aircraft snaps back when the clock runs.
bool TakeSetupEdit();

// Puts the last "(setup)" edit back to the value it started from. For when
// its Reset fails (a start the flight model cannot trim, such as an F-16
// at 100 000 ft): the world was kept as it was, so the edit must go too,
// or the panel and marker show a start the flight model never took, and
// every later Reset fails on it.
void RevertSetupEdit(ScenarioData& scenario, World& world);

}  // namespace ParameterPanel
