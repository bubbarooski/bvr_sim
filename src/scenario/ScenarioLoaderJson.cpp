#include "ScenarioLoaderJson.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <initializer_list>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "aircraft/AircraftEntity.h"
#include "components/AirborneRadarComponent.h"
#include "components/AutopilotComponent.h"
#include "components/GLimitComponent.h"
#include "components/JSBSimComponent.h"
#include "components/RadarCrossSection.h"
#include "components/RwrComponent.h"
#include "components/SpawnPose.h"
#include "components/Velocity.h"
#include "components/WaypointComponent.h"
#include "scenario/ParamRegistry.h"
#include "scenario/ScenarioEntity.h"
#include "sim/Environment.h"
#include "systems/FlightDynamicsSystem.h"
#include "util/Units.h"

namespace {

void RequireField(const nlohmann::json& j, const char* field) {
    if (!j.contains(field)) {
        throw std::runtime_error(std::string("Scenario missing required field: ") + field);
    }
}

// Every key of `block` must be one the loader reads, so a mistyped key
// (loopFrom for loop_from, missle_doctrine) fails here rather than loading a
// different scenario than the file describes.
void RejectUnknownKeys(const nlohmann::json& block, const std::string& where,
                       std::initializer_list<const char*> allowed) {
    if (!block.is_object()) return;
    for (auto it = block.begin(); it != block.end(); ++it) {
        bool known = false;
        for (const char* key : allowed) known = known || it.key() == key;
        if (!known) {
            throw std::runtime_error(where + " has an unknown key \"" + it.key() +
                                     "\" (docs/SCHEMA.md lists the keys)");
        }
    }
}

Vec3 ParseVec3(const nlohmann::json& arrayJson) {
    if (!arrayJson.is_array() || arrayJson.size() != 3) {
        throw std::runtime_error("expected a 3-element [x, y, z] array, got: " + arrayJson.dump());
    }
    return Vec3{arrayJson.at(0).get<double>(), arrayJson.at(1).get<double>(), arrayJson.at(2).get<double>()};
}

// A knob's JSON value, checked against the range the registry gives it (the
// range the GUI lets a user set, in the JSON's own unit). Outside it is a
// value nobody meant -- a negative nav gain, a wind that reverses -- so it is
// refused rather than flown.
// Feet and degrees; two statements of one start condition differing by less
// are rounding, not an edit.
constexpr double kStartTolerance = 1e-6;

double InRange(const ParamRegistry::ParamDef& param, const nlohmann::json& value) {
    double v = value.get<double>();
    if (!(v >= param.minValue && v <= param.maxValue)) {
        throw std::runtime_error(std::string(param.pointer) + " = " + value.dump() + " is outside its range " +
                                 nlohmann::json(param.minValue).dump() + " to " +
                                 nlohmann::json(param.maxValue).dump() + (*param.unit ? " " : "") + param.unit);
    }
    return v;
}

// Every scenario-wide field (missile_doctrine, environment) is a
// ParamRegistry entry, so the registry is the one list of what those
// blocks contain and how each field converts: the loader, the saver, the
// GUI editor and batch overrides cannot disagree. Each field is optional
// and keeps its ScenarioData default when absent, so a scenario can
// override one field of a block without respecifying the rest.
void LoadScenarioWide(const nlohmann::json& j, ScenarioData& data, World& world, ParamRegistry::ParamOwner owner) {
    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        if (param.owner != owner) continue;
        nlohmann::json::json_pointer pointer(param.pointer);
        if (!j.contains(pointer)) continue;
        const nlohmann::json& value = j.at(pointer);

        switch (param.type) {
            case ParamRegistry::ParamType::Double:
                *param.dbl(data, world, kNoEntity) = param.jsonScale.ToInternal(InRange(param, value));
                break;
            case ParamRegistry::ParamType::Bool:
                *param.flag(data, world, kNoEntity) = value.get<bool>();
                break;
            case ParamRegistry::ParamType::Enum: {
                std::string name = value.get<std::string>();
                int index = ParamRegistry::EnumIndex(param, name);
                if (index < 0) {
                    std::string allowed;
                    for (const char* option : param.enumAccess.names) {
                        allowed += (allowed.empty() ? "\"" : ", \"") + std::string(option) + "\"";
                    }
                    throw std::runtime_error(std::string("Scenario ") + param.pointer + " must be one of " + allowed +
                                             ", got: " + name);
                }
                param.enumAccess.set(data, world, kNoEntity, index);
                break;
            }
        }
    }
}

// Every leaf key of `block` (addressed by `blockPointer`, a registry
// pointer template) must be a teaching knob, or one of the block's
// structural keys. A type value is rejected naming the file it lives in, and
// an unknown key as not a parameter at all -- rather than either being
// silently ignored, so a mistyped scenario says what is wrong.
void RejectNonKnobKeys(const nlohmann::json& block, const std::string& blockPointer, const std::string& typeFile,
                       const std::vector<std::string>& structural = {}) {
    if (!block.is_object()) return;
    for (auto it = block.begin(); it != block.end(); ++it) {
        std::string pointer = blockPointer + "/" + it.key();
        if (std::find(structural.begin(), structural.end(), pointer) != structural.end()) continue;
        if (it.value().is_object() && ParamRegistry::Find(pointer) == nullptr) {
            RejectNonKnobKeys(it.value(), pointer, typeFile, structural);
            continue;
        }
        const ParamRegistry::ParamDef* param = ParamRegistry::Find(pointer);
        if (param == nullptr) {
            throw std::runtime_error(pointer + " is not a scenario parameter (docs/SCHEMA.md lists them)");
        }
        if (!param->knob) {
            throw std::runtime_error(pointer + " is the type's value, not a teaching knob: it is read-only" +
                                     (typeFile.empty() ? std::string() : " (it lives in " + typeFile + ")"));
        }
    }
}

// Writes every entity-owned registry field under `blockPrefix` (e.g.
// "/entities/{}/components/airborne_radar/") that the entity's components
// JSON sets, onto the already-attached component. Every entity field is a
// Double (ParamRegistryTable.EntityParamsAreDoubles). The registry is the one
// list of a block's keys, so the loader, saver and GUI agree on them.
void ApplyEntityOverrides(const nlohmann::json& componentsJson, const std::string& blockPrefix, ScenarioData& data,
                          World& world, EntityID entity) {
    const std::string componentsPrefix = "/entities/{}/components";
    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        if (param.owner != ParamRegistry::ParamOwner::Entity || param.type != ParamRegistry::ParamType::Double) {
            continue;
        }
        std::string pointer = param.pointer;
        if (pointer.rfind(blockPrefix, 0) != 0) continue;
        nlohmann::json::json_pointer relative(pointer.substr(componentsPrefix.size()));
        if (!componentsJson.contains(relative)) continue;
        *param.dbl(data, world, entity) = param.jsonScale.ToInternal(InRange(param, componentsJson.at(relative)));
    }
}

// The aircraft type's radar, then this aircraft's overrides of it. The
// radar hardware is the type's: a block here says "this aircraft's radar is
// on", and any key in it overrides the type for this one aircraft.
void LoadAirborneRadar(const nlohmann::json& componentsJson, const AircraftType& type, const std::string& name,
                       ScenarioData& data, World& world, EntityID entity) {
    const nlohmann::json& radarJson = componentsJson.at("airborne_radar");
    if (!type.radar.present) {
        throw std::runtime_error("entity '" + name + "': airborne_radar, but aircraft type '" + type.id +
                                 "' has no \"radar\" block");
    }
    RejectNonKnobKeys(radarJson, "/entities/{}/components/airborne_radar",
                      "data/aircraft/" + type.id + "/aircraft.json \"radar\"");
    world.Components<AirborneRadarComponent>().Add(entity, AircraftEntity::MakeAirborneRadar(type));
    ApplyEntityOverrides(componentsJson, "/entities/{}/components/airborne_radar/", data, world, entity);
}

// The aircraft type's radar signature: every aircraft of a type with a
// signature is radar-visible, and its shape is its type's alone. A
// "signature" block may set the one teaching knob, rcs_scale. A
// radar_cross_section block is rejected rather than ignored, so an aircraft
// never silently flies a different shape than its type's.
void LoadSignature(const nlohmann::json& componentsJson, const AircraftType& type, const std::string& name,
                   ScenarioData& data, World& world, EntityID entity) {
    if (componentsJson.contains("radar_cross_section")) {
        throw std::runtime_error("entity '" + name + "': radar_cross_section is not accepted on an aircraft: its "
                                 "signature is the type's (data/aircraft/" + type.id + "/aircraft.json \"signature\")");
    }
    if (!type.signature.present) {
        if (componentsJson.contains("signature")) {
            throw std::runtime_error("entity '" + name + "': signature, but aircraft type '" + type.id +
                                     "' has no \"signature\" block");
        }
        return;
    }
    if (componentsJson.contains("signature")) {
        RejectNonKnobKeys(componentsJson.at("signature"), "/entities/{}/components/signature",
                          "data/aircraft/" + type.id + "/aircraft.json \"signature\"");
    }
    world.Components<RadarCrossSection>().Add(entity, AircraftEntity::MakeRadarCrossSection(type));
    ApplyEntityOverrides(componentsJson, "/entities/{}/components/signature/", data, world, entity);
}

// "evasion" attaches the aircraft type's RWR, plus an empty
// WaypointComponent if the entity has no route yet. It attaches no
// decision-making: the RWR is a sensor, and what reacts to it is whatever
// external controller drives the waypoint queue.
void LoadRwr(const nlohmann::json& componentsJson, const AircraftType& type, const std::string& name,
             World& world, EntityID entity) {
    const nlohmann::json& evasionJson = componentsJson.at("evasion");
    RequireField(evasionJson, "rwr");
    const nlohmann::json& rwrJson = evasionJson.at("rwr");
    RejectUnknownKeys(rwrJson, "evasion.rwr", {});
    if (!type.rwr.present) {
        throw std::runtime_error("entity '" + name + "': evasion.rwr, but aircraft type '" + type.id +
                                 "' has no \"rwr\" block");
    }
    RejectNonKnobKeys(evasionJson, "/entities/{}/components/evasion",
                      "data/aircraft/" + type.id + "/aircraft.json \"rwr\"");
    world.Components<RwrComponent>().Add(entity, AircraftEntity::MakeRwr(type));

    if (!world.Components<WaypointComponent>().Has(entity)) {
        world.Components<WaypointComponent>().Add(entity, WaypointComponent{});
    }
}

}  // namespace

namespace ScenarioLoader {

ScenarioData LoadFromJson(const nlohmann::json& j, World& world, const AircraftCatalog& catalog,
                          const MissileCatalog& missiles) {
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    ComponentStore<Tag>& tags = world.Components<Tag>();
    RequireField(j, "seed");
    RequireField(j, "sim_speed");
    RequireField(j, "entities");

    ScenarioData data;
    // A negative seed would wrap to 2^64 - n and a fraction would truncate,
    // each running a different seed than the file says.
    const nlohmann::json& seedJson = j.at("seed");
    if (!seedJson.is_number_unsigned() && !(seedJson.is_number_integer() && seedJson.get<int64_t>() >= 0)) {
        throw std::runtime_error("seed must be a non-negative integer, got: " + seedJson.dump());
    }
    data.seed = seedJson.get<uint64_t>();
    data.simSpeed = j.at("sim_speed").get<double>();
    // A clock that never advances is a run that never starts.
    if (!(data.simSpeed > 0.0)) {
        throw std::runtime_error("sim_speed must be greater than 0, got: " + j.at("sim_speed").dump());
    }
    if (j.contains("description")) {
        if (!j.at("description").is_string()) throw std::runtime_error("description must be a string");
        data.description = j.at("description").get<std::string>();
    }

    // Optional: a scenario with no missile engagement omits it entirely.
    if (j.contains("missile_doctrine")) {
        const auto& doctrineJson = j.at("missile_doctrine");
        RequireField(doctrineJson, "kill_radius_m");
        RequireField(doctrineJson, "max_range_m");
        RequireField(doctrineJson, "nav_gain");
        if (!doctrineJson.contains("missile")) {
            throw std::runtime_error("Scenario missing required field: missile_doctrine.missile, a missile type under "
                                     "data/missiles/ (e.g. \"aim120\")");
        }
        // The type first: the doctrine's own values override it.
        data.SetMissileType(missiles.Get(doctrineJson.at("missile").get<std::string>()));
        RejectNonKnobKeys(doctrineJson, "/missile_doctrine",
                          "data/missiles/" + doctrineJson.at("missile").get<std::string>() + "/missile.json",
                          {"/missile_doctrine/missile"});
        LoadScenarioWide(j, data, world, ParamRegistry::ParamOwner::Doctrine);
    }

    // Before any entity: aircraft trim in this air at spawn.
    if (j.contains("environment")) RejectNonKnobKeys(j.at("environment"), "/environment", "");
    LoadScenarioWide(j, data, world, ParamRegistry::ParamOwner::Environment);

    RejectUnknownKeys(j, "scenario", {"seed", "sim_speed", "entities", "description", "missile_doctrine", "environment"});

    auto loadEntity = [&](const nlohmann::json& entityJson) {
        RequireField(entityJson, "name");
        RequireField(entityJson, "type");
        RequireField(entityJson, "transform");
        RequireField(entityJson, "components");
        RejectUnknownKeys(entityJson, "entity", {"name", "type", "transform", "components"});

        const auto& transformJson = entityJson.at("transform");
        RequireField(transformJson, "pos");
        RequireField(transformJson, "heading");
        RejectUnknownKeys(transformJson, "transform", {"pos", "heading"});

        EntityID entity = world.CreateEntity();

        Transform transform;
        transform.pos = ParseVec3(transformJson.at("pos"));
        transform.euler = Vec3{0.0, 0.0, transformJson.at("heading").get<double>()};
        transforms.Add(entity, transform);
        world.Components<SpawnPose>().Add(entity, SpawnPose{transform.pos, transform.euler.z()});

        tags.Add(entity, Tag{entityJson.at("name").get<std::string>()});

        // "components" is an open bag: every block below is optional, and
        // an entity simply never gets that component if it is absent.
        const auto& componentsJson = entityJson.at("components");
        if (componentsJson.contains("velocity")) {
            RejectUnknownKeys(componentsJson.at("velocity"), "velocity", {"linear"});
            Velocity velocity;
            velocity.linear = ParseVec3(componentsJson.at("velocity").at("linear"));
            world.Components<Velocity>().Add(entity, velocity);
        }

        // An aircraft's signature is its type's (attached after the jsbsim
        // block below); only an entity with no aircraft type spells out
        // its whole radar cross-section here.
        if (componentsJson.contains("radar_cross_section") && !componentsJson.contains("jsbsim")) {
            const auto& rcsJson = componentsJson.at("radar_cross_section");
            std::string entityName = entityJson.at("name").get<std::string>();
            RequireField(rcsJson, "static_rcs_m2");
            RequireField(rcsJson, "fluctuation_model");
            RejectUnknownKeys(rcsJson, "radar_cross_section", {"static_rcs_m2", "fluctuation_model"});

            RadarCrossSection rcs;
            rcs.staticRcsM2 = rcsJson.at("static_rcs_m2").get<double>();
            if (!(rcs.staticRcsM2 >= 0.0)) {
                throw std::runtime_error("radar_cross_section.static_rcs_m2 must be 0 or more, got: " +
                                         rcsJson.at("static_rcs_m2").dump());
            }
            rcs.fluctuation = ParseFluctuationModel(rcsJson.at("fluctuation_model").get<std::string>(),
                                                    "entity '" + entityName + "': radar_cross_section");
            world.Components<RadarCrossSection>().Add(entity, rcs);
            // The one teaching knob an entity with no type can carry. It is
            // shown and saved like an aircraft's, so it must load like one.
            if (componentsJson.contains("signature")) {
                RejectNonKnobKeys(componentsJson.at("signature"), "/entities/{}/components/signature", "");
                ApplyEntityOverrides(componentsJson, "/entities/{}/components/signature/", data, world, entity);
            }
        }

        // "points" are plain feet, unlike arrival_radius_m -- only the
        // radius is naturally metric. "speeds_kts" runs parallel to
        // "points" and may be shorter; a null or missing entry means that
        // point does not override speed.
        if (componentsJson.contains("waypoints")) {
            const auto& waypointsJson = componentsJson.at("waypoints");
            RequireField(waypointsJson, "points");
            RejectUnknownKeys(waypointsJson, "waypoints",
                              {"points", "speeds_kts", "arrival_radius_m", "loop", "loop_from"});
            const auto& pointsJson = waypointsJson.at("points");
            const nlohmann::json* speedsJson =
                waypointsJson.contains("speeds_kts") ? &waypointsJson.at("speeds_kts") : nullptr;

            WaypointComponent waypoint;
            for (size_t i = 0; i < pointsJson.size(); ++i) {
                WaypointPoint point;
                point.pos = ParseVec3(pointsJson.at(i));
                if (speedsJson && i < speedsJson->size() && !speedsJson->at(i).is_null()) {
                    point.speedKts = speedsJson->at(i).get<double>();
                }
                waypoint.queue.push_back(point);
            }
            waypoint.arrivalRadiusM = waypointsJson.value("arrival_radius_m", waypoint.arrivalRadiusM);
            // A radius of zero or less is a waypoint no aircraft can reach.
            if (!(waypoint.arrivalRadiusM > 0.0)) {
                throw std::runtime_error("waypoints.arrival_radius_m must be greater than 0, got: " +
                                         std::to_string(waypoint.arrivalRadiusM));
            }
            waypoint.loop = waypointsJson.value("loop", false);
            if (waypointsJson.contains("loop_from")) {
                int loopFrom = waypointsJson.at("loop_from").get<int>();
                if (!waypoint.loop) throw std::runtime_error("waypoints.loop_from needs \"loop\": true");
                if (loopFrom < 0 || static_cast<size_t>(loopFrom) >= waypoint.queue.size()) {
                    throw std::runtime_error("waypoints.loop_from must index one of the route's points (0 to " +
                                             std::to_string(waypoint.queue.size()) + " - 1)");
                }
                waypoint.loopFrom = static_cast<size_t>(loopFrom);
            }
            world.Components<WaypointComponent>().Add(entity, waypoint);
        }

        if (componentsJson.contains("jsbsim")) {
            const auto& jsbsimJson = componentsJson.at("jsbsim");
            // The flight model is the aircraft type's; naming it per scenario
            // would let two F-16s disagree about what an F-16 is.
            if (jsbsimJson.contains("model")) {
                throw std::runtime_error("jsbsim.model is not accepted: name an aircraft type with "
                                         "jsbsim.aircraft (a type under data/aircraft/, e.g. \"f16c_block50\")");
            }
            RequireField(jsbsimJson, "aircraft");
            RequireField(jsbsimJson, "altitude_ft");
            RequireField(jsbsimJson, "heading_deg");
            RequireField(jsbsimJson, "airspeed_kts");

            ScenarioEntity scenarioEntity;
            scenarioEntity.name = entityJson.at("name").get<std::string>();
            scenarioEntity.pos = transform.pos;
            scenarioEntity.jsbsim.type = catalog.Get(jsbsimJson.at("aircraft").get<std::string>());
            scenarioEntity.jsbsim.altitudeFt = jsbsimJson.at("altitude_ft").get<double>();
            scenarioEntity.jsbsim.headingDeg = jsbsimJson.at("heading_deg").get<double>();
            scenarioEntity.jsbsim.airspeedKts = jsbsimJson.at("airspeed_kts").get<double>();
            // Start altitude and heading are written twice: the flight model
            // starts from jsbsim.*, while the saver and a placement edit
            // change the transform. Left free to differ, an edit to one
            // would start the aircraft where the file does not say.
            if (std::fabs(transform.pos.y() - scenarioEntity.jsbsim.altitudeFt) > kStartTolerance) {
                throw std::runtime_error("entity '" + scenarioEntity.name + "': transform.pos[1] (" +
                                         std::to_string(transform.pos.y()) + ") and jsbsim.altitude_ft (" +
                                         std::to_string(scenarioEntity.jsbsim.altitudeFt) +
                                         ") are one start altitude and must be equal");
            }
            if (std::fabs(std::remainder(transform.euler.z() - scenarioEntity.jsbsim.headingDeg, 360.0)) >
                kStartTolerance) {
                throw std::runtime_error("entity '" + scenarioEntity.name + "': transform.heading (" +
                                         std::to_string(transform.euler.z()) + ") and jsbsim.heading_deg (" +
                                         std::to_string(scenarioEntity.jsbsim.headingDeg) +
                                         ") are one start heading and must be equal");
            }
            scenarioEntity.jsbsim.temperatureOffsetC = data.environment.temperatureOffsetC;
            // Attaches JSBSimComponent internally; throws if the model
            // fails to load, or if it cannot be trimmed.
            FlightDynamicsSystem::Init(world, entity, scenarioEntity);

            RejectUnknownKeys(jsbsimJson, "jsbsim", {"aircraft", "altitude_ft", "heading_deg", "airspeed_kts"});

            // Straight-and-level: autopilot targets are just the IC values.
            AutopilotComponent autopilot;
            autopilot.targetHeadingDeg = scenarioEntity.jsbsim.headingDeg;
            autopilot.targetAltitudeFt = scenarioEntity.jsbsim.altitudeFt;
            autopilot.targetSpeedKts = scenarioEntity.jsbsim.airspeedKts;
            world.Components<AutopilotComponent>().Add(entity, autopilot);

            // Every aircraft flies its type's envelope; none of its numbers is a
            // teaching knob.
            GLimitComponent gLimit = AircraftEntity::MakeGLimit(*scenarioEntity.jsbsim.type);
            world.Components<GLimitComponent>().Add(entity, gLimit);

            LoadSignature(componentsJson, *scenarioEntity.jsbsim.type, scenarioEntity.name, data, world, entity);
            if (componentsJson.contains("airborne_radar")) {
                LoadAirborneRadar(componentsJson, *scenarioEntity.jsbsim.type, scenarioEntity.name, data, world,
                                  entity);
            }
            if (componentsJson.contains("evasion")) {
                LoadRwr(componentsJson, *scenarioEntity.jsbsim.type, scenarioEntity.name, world, entity);
            }
        } else {
            for (const char* block : {"airborne_radar", "evasion"}) {
                if (componentsJson.contains(block)) {
                    throw std::runtime_error("entity '" + entityJson.at("name").get<std::string>() + "': " + block +
                                             " needs an aircraft (a jsbsim block naming its type) -- the sensor is "
                                             "the aircraft type's");
                }
            }
        }
        if (componentsJson.contains("signature") && !componentsJson.contains("jsbsim") &&
            !componentsJson.contains("radar_cross_section")) {
            throw std::runtime_error("entity '" + entityJson.at("name").get<std::string>() +
                                     "': signature needs an aircraft (a jsbsim block) or a radar_cross_section to scale");
        }
        // Last, so a rejected block reports its own message first.
        RejectUnknownKeys(componentsJson, "components",
                          {"velocity", "radar_cross_section", "waypoints", "jsbsim", "airborne_radar", "evasion",
                           "signature"});
    };

    // Everything that finds an entity by name (the saver, the launcher's
    // Pursuer/Target lookup, the GUI) takes the first match, so a repeated
    // name would silently alias the second entity onto the first.
    std::set<std::string> names;
    for (const auto& entityJson : j.at("entities")) {
        std::string name = entityJson.contains("name") && entityJson.at("name").is_string()
                               ? entityJson.at("name").get<std::string>()
                               : std::string("<unnamed>");
        if (!names.insert(name).second) {
            throw std::runtime_error("entity name '" + name +
                                     "' is used by more than one entity: names must be unique");
        }
        // Say which entity a bare parse error (index out of range, wrong
        // type) belongs to; a message that already names one is left alone.
        try {
            loadEntity(entityJson);
        } catch (const std::exception& e) {
            if (std::string(e.what()).find("entity '") != std::string::npos) throw;
            throw std::runtime_error("entity '" + name + "': " + e.what());
        }
    }

    return data;
}

}  // namespace ScenarioLoader
