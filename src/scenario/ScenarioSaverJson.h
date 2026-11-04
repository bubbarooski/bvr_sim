#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"

// Apart from ScenarioSaver.h to keep json.hpp out of the GUI translation
// units (see ScenarioLoaderJson.h).
namespace ScenarioSaver {

// The scenario exactly as currently edited: the file at sourcePath, with
// every live waypoint queue and every ParamRegistry value that differs
// from what that file would load patched in.
//
// The single definition of "the scenario as currently edited": Save writes
// it and Reset reloads from it, so the two cannot drift.
bool BuildJson(const std::string& sourcePath, World& world, ScenarioData& scenario, nlohmann::json& out,
               std::string& errorOut);

}  // namespace ScenarioSaver
