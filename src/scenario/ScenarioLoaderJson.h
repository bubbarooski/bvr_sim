#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "aircraft/AircraftCatalog.h"
#include "missile/MissileCatalog.h"
#include "scenario/ScenarioLoader.h"

// Apart from ScenarioLoader.h because deps/json vendors only the
// amalgamated json.hpp (no json_fwd.hpp): declaring this next to Load()
// would pull it into every translation unit that just wants ScenarioData.
namespace ScenarioLoader {

// The actual scenario parser. Load() is a thin wrapper that reads a file
// into a json and calls this; callers that build or modify a scenario in
// memory (the batch runner applying per-run overrides) call it directly
// and never touch the filesystem. Aircraft types named by the scenario
// resolve through `catalog`, its missile type through `missiles`.
ScenarioData LoadFromJson(const nlohmann::json& j, World& world,
                          const AircraftCatalog& catalog = AircraftCatalog::Default(),
                          const MissileCatalog& missiles = MissileCatalog::Default());

// Reads and parses a JSON file. Throws std::runtime_error if it cannot be
// opened, and nlohmann's parse_error if it is not valid JSON.
nlohmann::json ReadJsonFile(const std::string& path);

}  // namespace ScenarioLoader
