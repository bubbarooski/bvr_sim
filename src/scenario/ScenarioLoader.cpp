#include "ScenarioLoader.h"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "scenario/ScenarioLoaderJson.h"

namespace ScenarioLoader {

nlohmann::json ReadJsonFile(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) throw std::runtime_error("file not found: " + path);
    nlohmann::json j;
    file >> j;
    return j;
}

ScenarioData Load(const std::string& path, World& world) { return LoadFromJson(ReadJsonFile(path), world); }

}  // namespace ScenarioLoader
