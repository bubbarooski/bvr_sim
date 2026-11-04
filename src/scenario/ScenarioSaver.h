#pragma once

#include <string>

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"

// Writes the scenario as currently edited -- live waypoint queues plus
// every ParamRegistry value the user has changed -- patched into a fresh
// copy of the source file.
//
// Deliberately not a general-purpose World-to-JSON serializer: several
// fields (the aircraft model name, entity type) are never stored in any
// live component, so a full round-trip from World state alone is
// impossible. Re-reading the source file is what sidesteps that, and is
// also what keeps a saved scenario's content limited to what actually
// changed. The file is re-serialised, so its keys come out in alphabetical
// order and its indent is normalised: the values are the source's, the
// layout is not.
namespace ScenarioSaver {

// Returns true on success. On failure returns false and sets errorOut
// (source file missing/unparsable, destPath not writable, ...).
bool Save(const std::string& sourcePath, const std::string& destPath, World& world, ScenarioData& scenario,
          std::string& errorOut);

// The file name Save As writes for a typed name: "<name>.json", a typed
// ".json" kept. Letters, digits, '-' and '_' only, so the name cannot
// leave scenarios/custom/; a leading '_' is reserved for the template.
// Returns "" and sets errorOut when the name is refused.
std::string CustomFileName(const std::string& typed, std::string& errorOut);

// Rebuilds the world from the scenario as currently edited: the Reset
// button's core. Everything the user changed survives; everything the run
// produced does not.
//
// Destroys every entity and reloads, so the caller must reset its own
// per-run state afterwards (RNG seed, event log, clock, trails) exactly
// as it would after a file reload. `scenario` is overwritten with the
// reloaded data.
bool ResetWorld(const std::string& sourcePath, World& world, ScenarioData& scenario, std::string& errorOut);

}  // namespace ScenarioSaver
