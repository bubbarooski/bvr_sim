#include "ScenarioSaver.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "aircraft/AircraftEntity.h"
#include "components/AirborneRadarComponent.h"
#include "components/AutopilotComponent.h"
#include "components/GLimitComponent.h"
#include "components/RadarCrossSection.h"
#include "components/RwrComponent.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "components/WaypointComponent.h"
#include "ecs/ComponentStore.h"
#include "scenario/ParamRegistry.h"
#include "scenario/ScenarioLoaderJson.h"
#include "scenario/ScenarioSaverJson.h"
#include "util/EntityLookup.h"

namespace {

// A world holding one entity with a default-constructed instance of every
// component the registry touches.
//
// This is how "what would the source file load for this field?" is
// answered when the field is absent from the JSON: the loader would leave
// the C++ default, and the accessor reads it from here. Cheap by
// construction -- no scenario parse, no JSBSim, no trim -- which is what
// makes writing only changed values affordable on every save.
struct Defaults {
    World world;
    ScenarioData scenario;
    EntityID entity;

    Defaults() {
        entity = world.CreateEntity();
        world.Components<Transform>().Add(entity, Transform{});
        world.Components<AutopilotComponent>().Add(entity, AutopilotComponent{});
        world.Components<GLimitComponent>().Add(entity, GLimitComponent{});
        world.Components<RadarCrossSection>().Add(entity, RadarCrossSection{});
        world.Components<AirborneRadarComponent>().Add(entity, AirborneRadarComponent{});
        world.Components<RwrComponent>().Add(entity, RwrComponent{});
    }

    // An aircraft's envelope defaults to its type's, not to the
    // component's empty one: a field the file omits loads as the type's
    // value, so that is the baseline an edit is measured against. Called
    // per subject before patching its entity-scoped fields.
    void MatchAircraftType(World& subjectWorld, EntityID subject) {
        const AircraftType* type = AircraftEntity::TypeOf(subjectWorld, subject);
        world.Components<GLimitComponent>().Get(entity) =
            type != nullptr ? AircraftEntity::MakeGLimit(*type) : GLimitComponent{};
        // Likewise its radar: a field the airborne_radar block omits loads
        // as the type's.
        world.Components<AirborneRadarComponent>().Get(entity) =
            type != nullptr && type->radar.present ? AircraftEntity::MakeAirborneRadar(*type)
                                                   : AirborneRadarComponent{};
        // And its RWR's receiver.
        world.Components<RwrComponent>().Get(entity) =
            type != nullptr && type->rwr.present ? AircraftEntity::MakeRwr(*type) : RwrComponent{};
        // And its radar signature.
        world.Components<RadarCrossSection>().Get(entity) =
            type != nullptr && type->signature.present ? AircraftEntity::MakeRadarCrossSection(*type)
                                                       : RadarCrossSection{};
    }
};

// Relative comparison, so a slider dragged away and back does not write a
// key that differs only in the last bit. Far below any meaningful change
// to a physical parameter.
constexpr double kSameRelativeTolerance = 1e-12;

bool SameDouble(double a, double b) {
    double scale = std::max({1.0, std::fabs(a), std::fabs(b)});
    return std::fabs(a - b) <= kSameRelativeTolerance * scale;
}

// Every scenario file is written with this indent.
constexpr int kJsonIndent = 4;

// One subject a registry entry can apply to: an entity, paired with its
// index in the source JSON's own array.
struct Subject {
    EntityID entity = 0;
    int index = -1;
};

// Pairs each element of `arrayKey` with the live entity of the same tag
// name. Elements whose entity is missing from the world are skipped --
// they cannot have been edited. The loader requires every element's
// "name".
std::vector<Subject> CollectSubjects(const nlohmann::json& j, World& world, const char* arrayKey) {
    std::vector<Subject> subjects;
    if (!j.contains(arrayKey)) return subjects;
    int index = 0;
    for (const nlohmann::json& element : j.at(arrayKey)) {
        std::optional<EntityID> entity = EntityLookup::FindByTagName(world, element.at("name").get<std::string>());
        if (entity.has_value()) subjects.push_back(Subject{*entity, index});
        ++index;
    }
    return subjects;
}

// Writes `param` for one subject if its live value differs from what the
// source file would load for it.
void PatchParam(nlohmann::json& j, const nlohmann::json& source, const ParamRegistry::ParamDef& param,
                World& world, ScenarioData& scenario, Defaults& defaults, const Subject& subject) {
    // A type value cannot be edited, and a scenario key for one would not
    // load.
    if (!param.knob) return;
    std::string pointerText = ParamRegistry::ResolvePointer(param, subject.index);
    nlohmann::json::json_pointer pointer(pointerText);

    bool inSource = source.contains(pointer);
    EntityID defaultEntity = defaults.entity;

    switch (param.type) {
        case ParamRegistry::ParamType::Double: {
            double* live = param.dbl(scenario, world, subject.entity);
            if (live == nullptr) return;
            double baseline;
            if (inSource) {
                baseline = param.jsonScale.ToInternal(source.at(pointer).get<double>());
            } else {
                double* def = param.dbl(defaults.scenario, defaults.world, defaultEntity);
                if (def == nullptr) return;
                baseline = *def;
            }
            if (SameDouble(*live, baseline)) return;
            // JSON has no NaN or infinity: dumped they become null, which
            // the loader rejects, so the file would never load again.
            if (!std::isfinite(*live)) {
                throw std::runtime_error(pointerText + " is not a finite number, so it cannot be saved");
            }
            j[pointer] = param.jsonScale.ToJson(*live);
            return;
        }
        case ParamRegistry::ParamType::Bool: {
            bool* live = param.flag(scenario, world, subject.entity);
            if (live == nullptr) return;
            bool baseline;
            if (inSource) {
                baseline = source.at(pointer).get<bool>();
            } else {
                bool* def = param.flag(defaults.scenario, defaults.world, defaultEntity);
                if (def == nullptr) return;
                baseline = *def;
            }
            if (*live == baseline) return;
            j[pointer] = *live;
            return;
        }
        case ParamRegistry::ParamType::Enum: {
            int live = param.enumAccess.get(scenario, world, subject.entity);
            if (live < 0) return;  // component absent
            int baseline;
            if (inSource) {
                baseline = ParamRegistry::EnumIndex(param, source.at(pointer).get<std::string>());
            } else {
                baseline = param.enumAccess.get(defaults.scenario, defaults.world, defaultEntity);
            }
            if (live == baseline) return;
            j[pointer] = param.enumAccess.names[static_cast<size_t>(live)];
            return;
        }
    }
}

// The route as authored, not just what is still ahead: points already
// reached come first, and a looping queue is rotated back to its original
// start. Without this a save or Reset mid-run silently drops or reorders
// the route. A partial loop's queue holds only the repeating part once
// its lead-in is passed, and loopRotation counts only its rotations, so
// the same un-rotation applies.
std::vector<WaypointPoint> FullRoute(const WaypointComponent& waypoint) {
    std::vector<WaypointPoint> route(waypoint.passed.begin(), waypoint.passed.end());
    size_t n = waypoint.queue.size();
    size_t shift = n > 0 ? waypoint.loopRotation % n : 0;
    for (size_t i = 0; i < n; ++i) route.push_back(waypoint.queue[(i + n - shift) % n]);
    return route;
}

void PatchWaypoints(nlohmann::json& j, World& world) {
    ComponentStore<WaypointComponent>& waypointStore = world.Components<WaypointComponent>();
    for (auto& entityJson : j.at("entities")) {
        if (!entityJson.contains("name")) continue;
        std::optional<EntityID> entity = EntityLookup::FindByTagName(world, entityJson.at("name").get<std::string>());
        if (!entity.has_value()) continue;
        const WaypointComponent* waypointPtr = waypointStore.TryGet(*entity);
        if (waypointPtr == nullptr) continue;
        const WaypointComponent& waypoint = *waypointPtr;

        // The loader gives every evasion entity an empty route even when
        // the file has none. Writing that back would add an empty block
        // nobody authored; a route emptied by editing is still written.
        std::vector<WaypointPoint> route = FullRoute(waypoint);
        bool hadWaypoints = entityJson.contains("components") && entityJson.at("components").contains("waypoints");
        if (route.empty() && !hadWaypoints) continue;

        // Matches ScenarioLoaderJson.cpp's read schema exactly: "points" in
        // plain feet, "speeds_kts" parallel to points (omitted entirely
        // if every point leaves speed unset, null per-point otherwise --
        // same "unset -> leave targetSpeedKts alone" convention).
        nlohmann::json pointsJson = nlohmann::json::array();
        nlohmann::json speedsJson = nlohmann::json::array();
        bool anySpeed = false;
        for (const WaypointPoint& point : route) {
            if (!point.pos.allFinite() || (point.speedKts.has_value() && !std::isfinite(*point.speedKts))) {
                throw std::runtime_error("entity '" + entityJson.at("name").get<std::string>() +
                                         "' has a waypoint that is not a finite number, so it cannot be saved");
            }
            pointsJson.push_back(nlohmann::json::array({point.pos.x(), point.pos.y(), point.pos.z()}));
            if (point.speedKts.has_value()) {
                speedsJson.push_back(*point.speedKts);
                anySpeed = true;
            } else {
                speedsJson.push_back(nullptr);
            }
        }

        nlohmann::json waypointsJson;
        waypointsJson["points"] = pointsJson;
        if (anySpeed) waypointsJson["speeds_kts"] = speedsJson;
        waypointsJson["arrival_radius_m"] = waypoint.arrivalRadiusM;
        waypointsJson["loop"] = waypoint.loop;
        // Only a partial loop writes it, so files without the key keep
        // their exact shape. Clamped: a route shortened by editing must
        // still load.
        if (waypoint.loop && waypoint.loopFrom > 0 && waypoint.loopFrom < route.size()) {
            waypointsJson["loop_from"] = waypoint.loopFrom;
        }

        // Replaces only the "waypoints" key; every other component block on
        // the entity is left untouched.
        if (!entityJson.contains("components")) entityJson["components"] = nlohmann::json::object();
        entityJson["components"]["waypoints"] = waypointsJson;
    }
}

// Writes every live edit into `j`, which holds the source file's content on
// entry. Throws if a live value cannot be written (not a finite number).
void PatchEdits(nlohmann::json& j, World& world, ScenarioData& scenario) {
    // Compared against throughout, so later patches cannot shift the
    // baseline for earlier ones.
    const nlohmann::json source = j;

    PatchWaypoints(j, world);

    Defaults defaults;
    // Likewise a doctrine field the file omits loads as the missile type's
    // value, so that is its baseline.
    if (scenario.missileType != nullptr) defaults.scenario.SetMissileType(scenario.missileType);
    std::vector<Subject> entities = CollectSubjects(source, world, "entities");

    // Doctrine is only patched into a scenario that already has a
    // missile_doctrine block. Creating one from a single edited key would
    // leave it without kill_radius_m / max_range_m / nav_gain, which the
    // loader requires -- an unloadable file. A scenario with no missile
    // has no doctrine worth tuning anyway.
    bool hasDoctrine = source.contains("missile_doctrine");

    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        switch (param.owner) {
            case ParamRegistry::ParamOwner::Doctrine:
                if (hasDoctrine) PatchParam(j, source, param, world, scenario, defaults, Subject{0, -1});
                break;
            // Every environment field is optional, so unlike doctrine a
            // block created from one edited key still loads.
            case ParamRegistry::ParamOwner::Environment:
                PatchParam(j, source, param, world, scenario, defaults, Subject{0, -1});
                break;
            case ParamRegistry::ParamOwner::Entity:
                for (const Subject& subject : entities) {
                    defaults.MatchAircraftType(world, subject.entity);
                    PatchParam(j, source, param, world, scenario, defaults, subject);
                }
                break;
        }
    }

    // The flight model starts from the jsbsim block's altitude and heading
    // (east/north come from the transform), so a placement edit to either
    // must carry into it or the aircraft would start where the file said.
    for (size_t i = 0; i < j["entities"].size() && i < source["entities"].size(); ++i) {
        nlohmann::json& entity = j["entities"][i];
        const nlohmann::json& before = source["entities"][i];
        if (!entity.contains("transform") || !entity.contains("components") ||
            !entity["components"].contains("jsbsim"))
            continue;
        nlohmann::json& jsbsim = entity["components"]["jsbsim"];
        const nlohmann::json& transform = entity["transform"];
        if (transform.contains("pos") && transform["pos"][1] != before["transform"]["pos"][1])
            jsbsim["altitude_ft"] = transform["pos"][1];
        if (transform.contains("heading") && transform["heading"] != before["transform"]["heading"])
            jsbsim["heading_deg"] = transform["heading"];
    }
}

}  // namespace

namespace ScenarioSaver {

bool BuildJson(const std::string& sourcePath, World& world, ScenarioData& scenario, nlohmann::json& out,
               std::string& errorOut) {
    nlohmann::json j;
    try {
        j = ScenarioLoader::ReadJsonFile(sourcePath);
    } catch (const std::exception& e) {
        errorOut = std::string("Failed to read source scenario: ") + e.what();
        return false;
    }

    if (!j.contains("entities")) {
        errorOut = "Source scenario has no \"entities\" array";
        return false;
    }

    try {
        PatchEdits(j, world, scenario);
    } catch (const std::exception& e) {
        errorOut = e.what();
        return false;
    }
    out = std::move(j);
    errorOut.clear();
    return true;
}

bool Save(const std::string& sourcePath, const std::string& destPath, World& world, ScenarioData& scenario,
          std::string& errorOut) {
    nlohmann::json j;
    if (!BuildJson(sourcePath, world, scenario, j, errorOut)) return false;

    std::ofstream outFile(destPath);
    if (!outFile.is_open()) {
        errorOut = "Could not open destination path for writing: " + destPath;
        return false;
    }
    outFile << j.dump(kJsonIndent);
    outFile.close();
    // A full disk or a vanished share fails here, after open() succeeded.
    if (outFile.fail()) {
        errorOut = "Could not finish writing: " + destPath;
        return false;
    }
    errorOut.clear();
    return true;
}

bool ResetWorld(const std::string& sourcePath, World& world, ScenarioData& scenario, std::string& errorOut) {
    nlohmann::json j;
    if (!BuildJson(sourcePath, world, scenario, j, errorOut)) return false;

    // Built into a scratch world first: BuildJson only lightly validates
    // `j`, and a loader failure partway through must not leave `world`
    // half-built.
    World fresh;
    ScenarioData freshScenario;
    try {
        freshScenario = ScenarioLoader::LoadFromJson(j, fresh);
    } catch (const std::exception& e) {
        errorOut = std::string("Failed to reload edited scenario: ") + e.what();
        return false;
    }
    world.ReplaceWith(std::move(fresh));
    scenario = std::move(freshScenario);
    errorOut.clear();
    return true;
}

std::string CustomFileName(const std::string& typed, std::string& errorOut) {
    const std::string kJsonSuffix = ".json";
    std::string stem = typed;
    if (stem.size() >= kJsonSuffix.size() &&
        stem.compare(stem.size() - kJsonSuffix.size(), kJsonSuffix.size(), kJsonSuffix) == 0)
        stem.resize(stem.size() - kJsonSuffix.size());

    if (stem.empty()) {
        errorOut = "Enter a filename first.";
        return "";
    }
    auto allowed = [](char c) { return std::isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_'; };
    if (!std::all_of(stem.begin(), stem.end(), allowed)) {
        errorOut = "Use letters, digits, - and _ only (no folders).";
        return "";
    }
    if (stem.front() == '_') {
        errorOut = "Names starting with _ are reserved for the template.";
        return "";
    }
    errorOut.clear();
    return stem + kJsonSuffix;
}

}  // namespace ScenarioSaver
