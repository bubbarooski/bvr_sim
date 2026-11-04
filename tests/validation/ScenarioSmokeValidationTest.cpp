// Headless full-pipeline sanity run for every teaching scenario (the
// numbered files directly under scenarios/). Deliberately excludes
// scenarios/custom/ so the gate depends only on the committed set, not on
// whatever the GUI has saved there. Drives SimLoop::Step, the same entry
// point the GUI uses, so this exercises the real pipeline rather than a
// reimplementation of it.
//
// Engagements (a Pursuer, a Target and a missile): expect a LAUNCH event
// and an eventual INTERCEPT or MISS -- the engagement must actually
// resolve, not hang with the missile forever "active". A scenario without
// a missile: the bar is just "runs to completion without an exception".
//
// The scenario list is discovered at runtime via a filesystem scan, so it
// can't be split into individually-named TEST() cases at compile time --
// one TEST() sweeps every discovered scenario, using SCOPED_TRACE so a
// failure still names the specific scenario path that failed.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "AircraftRig.h"
#include "JsbSimDebugSilencer.h"
#include "FGFDMExec.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGTank.h"
#include "aircraft/AircraftCatalog.h"
#include "components/AircraftComponent.h"
#include "components/GLimitComponent.h"
#include "components/GuidanceComponent.h"
#include "components/JSBSimComponent.h"
#include "components/MissileComponent.h"
#include "components/MissileGLimitComponent.h"
#include "components/RadarCrossSection.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/MissileCatalog.h"
#include "scenario/ScenarioLoader.h"
#include "scenario/ScenarioLoaderJson.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"
#include "sim/SimLoop.h"
#include "systems/LaunchSystem.h"
#include "util/EntityLookup.h"

namespace {

constexpr double kMaxSimTime = 400.0;  // covers the longest curated engagement with margin

std::vector<std::string> EnumerateScenarios() {
    std::vector<std::string> result;
    std::error_code ec;
    // The teaching set: the numbered files directly under scenarios/
    // (scenarios/custom/ is the user's, not swept).
    for (const auto& entry : std::filesystem::directory_iterator("scenarios", ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            result.push_back(entry.path().generic_string());
        }
    }
    std::sort(result.begin(), result.end());
    return result;
}

::testing::AssertionResult RunScenario(const std::string& path) {
    World world;
    SimContext simContext;

    ScenarioData scenario;
    try {
        scenario = ScenarioLoader::Load(path, world);
    } catch (const std::exception& e) {
        return ::testing::AssertionFailure() << "failed to load: " << e.what();
    }
    SimLoop::BeginRun(scenario, simContext, scenario.seed);

    bool isCompound = LaunchSystem::InitFromTags(world, scenario);

    bool launched = false, intercepted = false, missed = false;
    size_t eventsSeen = 0;
    double simTime = 0.0;
    try {
        while (simTime < kMaxSimTime) {
            SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
            const auto& events = simContext.eventLog.Events();
            for (; eventsSeen < events.size(); ++eventsSeen) {
                const std::string& type = events[eventsSeen].type;
                if (type == "LAUNCH") launched = true;
                if (type == "INTERCEPT") intercepted = true;
                if (type == "MISS") missed = true;
            }
            // Compound scenarios: stop once resolved -- no need to keep
            // simulating a dead missile/aircraft out to kMaxSimTime.
            if (isCompound && launched && (intercepted || missed)) break;
        }
    } catch (const std::exception& e) {
        return ::testing::AssertionFailure() << "exception at t=" << simTime << "s: " << e.what();
    }

    if (!isCompound) return ::testing::AssertionSuccess();

    if (launched && (intercepted || missed)) return ::testing::AssertionSuccess();
    return ::testing::AssertionFailure() << "launched=" << (launched ? "yes" : "no")
                                          << " intercepted=" << (intercepted ? "yes" : "no")
                                          << " missed=" << (missed ? "yes" : "no")
                                          << " (resolved at t=" << simTime << "s)";
}

TEST(ScenarioSmoke, AllCuratedScenariosRunCleanly) {
    std::vector<std::string> scenarios = EnumerateScenarios();
    ASSERT_FALSE(scenarios.empty()) << "no scenario files found under scenarios/ -- run this from the repo root";

    for (const std::string& path : scenarios) {
        SCOPED_TRACE(path);
        EXPECT_TRUE(RunScenario(path));
    }
}

// The environment is a temperature offset and one steady wind: any other
// key is a load error, not silently ignored.
TEST(ScenarioSmoke, LoaderRejectsUnknownEnvironmentKeys) {
    auto load = [](const nlohmann::json& environment) {
        nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
        json["environment"] = environment;
        World world;
        return ScenarioLoader::LoadFromJson(json, world);
    };
    EXPECT_NO_THROW(load({{"wind_speed_kts", 20.0}, {"wind_from_deg", 270.0}, {"temperature_offset_c", 10.0}}));
    EXPECT_THROW(load({{"humidity_pct", 50.0}}), std::runtime_error);
    EXPECT_THROW(load({{"wind_speed_kts", 20.0}, {"made_up_block", {{"x", 1.0}}}}), std::runtime_error);
}

// The optional "description" (the GUI's "what to watch" text) loads as
// written, is empty when absent, and must be a string.
TEST(ScenarioSmoke, LoaderReadsDescription) {
    nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    {
        World world;
        EXPECT_TRUE(ScenarioLoader::LoadFromJson(json, world).description.empty());
    }
    json["description"] = "Watch the cone turn amber.";
    {
        World world;
        EXPECT_EQ(ScenarioLoader::LoadFromJson(json, world).description, "Watch the cone turn amber.");
    }
    json["description"] = 3;
    World world;
    EXPECT_THROW(ScenarioLoader::LoadFromJson(json, world), std::runtime_error);
}

// A scenario aircraft spawns with its type's fuel load: a per-entity
// internal_fuel_fraction key is a load error. The fill itself stays for
// test rigs matching a published weight: it fills the F-16's two internal
// tanks (6 972 lb) and nothing else, before trim, and the aircraft still
// trims and flies level; out-of-range values throw.
TEST(ScenarioSmoke, InternalFuelFraction) {
    nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    for (auto& entity : json["entities"]) {
        if (entity["components"].contains("jsbsim")) {
            entity["components"]["jsbsim"]["internal_fuel_fraction"] = 1.0;
            break;
        }
    }
    World badWorld;
    EXPECT_THROW(ScenarioLoader::LoadFromJson(json, badWorld), std::runtime_error);

    struct Loaded {
        double weightLbs, internalLbs, externalLbs, altitudeErrorFt;
    };
    const double altitudeFt = 25000.0;
    auto load = [altitudeFt](std::optional<double> fraction) {
        testrig::AircraftRig rig(testrig::kF16, altitudeFt, 400.0, 0.0, {}, fraction);
        Loaded loaded{};
        loaded.weightLbs = rig.Prop("inertia/weight-lbs");
        loaded.internalLbs = rig.Prop("propulsion/tank[0]/contents-lbs") + rig.Prop("propulsion/tank[1]/contents-lbs");
        loaded.externalLbs = rig.Prop("propulsion/tank[2]/contents-lbs") + rig.Prop("propulsion/tank[3]/contents-lbs");
        rig.Fly(30.0);
        loaded.altitudeErrorFt = std::fabs(rig.Prop("position/h-sl-ft") - altitudeFt);
        return loaded;
    };

    Loaded modelDefault = load(std::nullopt);
    Loaded fullLoad = load(1.0);
    Loaded lowLoad = load(0.05);

    EXPECT_NEAR(modelDefault.internalLbs, 3000.0, 1.0);  // the model file's own load
    EXPECT_NEAR(fullLoad.internalLbs, 6972.0, 1.0);
    EXPECT_NEAR(lowLoad.internalLbs, 0.05 * 6972.0, 1.0);
    EXPECT_EQ(fullLoad.externalLbs, 0.0);
    EXPECT_NEAR(fullLoad.weightLbs - modelDefault.weightLbs, 3972.0, 1.0);
    // Trimmed at the loaded weight: level flight holds at either extreme.
    EXPECT_LT(fullLoad.altitudeErrorFt, 50.0);
    EXPECT_LT(lowLoad.altitudeErrorFt, 50.0);

    // Empty tanks leave no running engine to trim with.
    EXPECT_THROW(load(1.5), std::runtime_error);
    EXPECT_THROW(load(0.0), std::runtime_error);
}

}  // namespace

// Aircraft come from their type: an entity names a type under
// data/aircraft and flies its envelope, and a scenario overriding that
// envelope or still naming the JSBSim model directly is a load error.
TEST(ScenarioSmoke, AircraftTypeSuppliesTheEnvelope) {
    auto loadTarget = [](const std::function<void(nlohmann::json&)>& edit) {
        nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
        nlohmann::json& target = json["entities"][1]["components"];
        edit(target);
        auto world = std::make_unique<World>();
        ScenarioLoader::LoadFromJson(json, *world);
        return world;
    };
    const AircraftType& f16 = *AircraftCatalog::Default().Get("f16c_block50");

    // No g_limit block: the type's envelope, on a typed aircraft.
    auto plain = loadTarget([](nlohmann::json&) {});
    EntityID target = *EntityLookup::FindByTagName(*plain, "Target");
    EXPECT_EQ(plain->Components<AircraftComponent>().Get(target).type->id, "f16c_block50");
    const GLimitComponent& typeEnvelope = plain->Components<GLimitComponent>().Get(target);
    EXPECT_DOUBLE_EQ(typeEnvelope.clMax, f16.envelope.clMax);
    EXPECT_DOUBLE_EQ(typeEnvelope.wingAreaFt2, f16.wingAreaFt2);
    EXPECT_DOUBLE_EQ(typeEnvelope.structuralGLimitPos, f16.envelope.structuralGLimitPos);

    // The envelope is the type's alone: a g_limit block is a load error.
    EXPECT_THROW(loadTarget([](nlohmann::json& c) { c["g_limit"] = {{"structural_g_limit_pos", 4.0}}; }),
                 std::runtime_error);

    // The per-scenario model key, and a type that does not exist.
    EXPECT_THROW(loadTarget([](nlohmann::json& c) { c["jsbsim"]["model"] = "f16"; }), std::runtime_error);
    EXPECT_THROW(loadTarget([](nlohmann::json& c) { c["jsbsim"]["aircraft"] = "no_such_type"; }),
                 std::runtime_error);
}

// The missile comes from its type: the doctrine names a type under
// data/missiles, its values are the type's unless the doctrine overrides a
// field, the launched missile carries the type, and a doctrine naming no
// type is a load error.
TEST(ScenarioSmoke, MissileTypeSuppliesTheMissile) {
    auto load = [](const std::function<void(nlohmann::json&)>& edit, World& world) {
        nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
        edit(json["missile_doctrine"]);
        return ScenarioLoader::LoadFromJson(json, world);
    };
    const MissileType& aim120 = *MissileCatalog::Default().Get("aim120");

    World plainWorld;
    ScenarioData plain = load([](nlohmann::json&) {}, plainWorld);
    ASSERT_NE(plain.missileType, nullptr);
    EXPECT_EQ(plain.missileType->id, "aim120");
    EXPECT_DOUBLE_EQ(plain.missileBoostThrustLbf, aim120.propulsion.boostThrustLbf);
    EXPECT_DOUBLE_EQ(plain.missileSeekerFovHalfAngleDeg, aim120.seeker.fovHalfAngleDeg);
    EXPECT_EQ(plain.missileSeekerMode, aim120.seeker.mode);

    // The missile LaunchSystem creates is the type, with the type's fins.
    ASSERT_TRUE(LaunchSystem::InitFromTags(plainWorld, plain));
    EntityID missile = *EntityLookup::FindByTagName(plainWorld, "Missile");
    EXPECT_EQ(plainWorld.Components<MissileComponent>().Get(missile).type.get(), &aim120);
    EXPECT_DOUBLE_EQ(plainWorld.Components<MissileGLimitComponent>().Get(missile).structuralGLimit,
                     aim120.aero.structuralGLimit);
    EXPECT_DOUBLE_EQ(plainWorld.Components<GuidanceComponent>().Get(missile).dryMassSlug, aim120.mass.dryMassSlug);

    // A doctrine override replaces only what it names.
    World narrowWorld;
    ScenarioData narrow = load([](nlohmann::json& d) { d["seeker"] = {{"fov_half_angle_deg", 20.0}}; }, narrowWorld);
    EXPECT_DOUBLE_EQ(narrow.missileSeekerFovHalfAngleDeg, 20.0);
    EXPECT_DOUBLE_EQ(narrow.missileMemoryCoastTimeS, aim120.seeker.memoryCoastTimeS);

    // The response time is the type's; the launched missile carries it.
    EXPECT_DOUBLE_EQ(plainWorld.Components<GuidanceComponent>().Get(missile).responseTimeS,
                     aim120.autopilot.responseTimeS);

    // The self-destruct limits: the type's unless the doctrine overrides
    // one, and the launched missile carries what the doctrine says.
    EXPECT_DOUBLE_EQ(plain.missileSelfDestructMinAvailableG, aim120.selfDestruct.minAvailableG);
    EXPECT_DOUBLE_EQ(plainWorld.Components<GuidanceComponent>().Get(missile).selfDestructMaxTimeOfFlightS,
                     aim120.selfDestruct.maxTimeOfFlightS);
    World longWorld;
    ScenarioData longFlight =
        load([](nlohmann::json& d) { d["self_destruct"] = {{"max_time_of_flight_s", 300.0}}; }, longWorld);
    EXPECT_DOUBLE_EQ(longFlight.missileSelfDestructMaxTimeOfFlightS, 300.0);
    EXPECT_DOUBLE_EQ(longFlight.missileSelfDestructMinAvailableG, aim120.selfDestruct.minAvailableG);
    ASSERT_TRUE(LaunchSystem::InitFromTags(longWorld, longFlight));
    EXPECT_DOUBLE_EQ(longWorld.Components<GuidanceComponent>()
                         .Get(*EntityLookup::FindByTagName(longWorld, "Missile"))
                         .selfDestructMaxTimeOfFlightS,
                     300.0);

    // A key that is not a doctrine parameter is rejected.
    World badWorld;
    EXPECT_THROW(load([](nlohmann::json& d) { d["made_up_key"] = 1.0; }, badWorld), std::runtime_error);
    EXPECT_THROW(load([](nlohmann::json& d) { d["autopilot"] = {{"made_up_key", 1.0}}; }, badWorld),
                 std::runtime_error);
    EXPECT_THROW(load([](nlohmann::json& d) { d.erase("missile"); }, badWorld), std::runtime_error);
    EXPECT_THROW(load([](nlohmann::json& d) { d["missile"] = "no_such_missile"; }, badWorld), std::runtime_error);
}

// Only the teaching knobs are scenario keys. A type value is rejected, as
// is a key that is no parameter at all, in every overridable block --
// rather than either being silently ignored.
TEST(ScenarioSmoke, LoaderAcceptsOnlyTeachingKnobs) {
    auto load = [](const std::function<void(nlohmann::json&)>& edit) {
        nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_notch_beam.json");
        edit(json);
        World world;
        return ScenarioLoader::LoadFromJson(json, world);
    };
    auto pursuer = [](nlohmann::json& j) -> nlohmann::json& { return j["entities"][0]["components"]; };
    auto target = [](nlohmann::json& j) -> nlohmann::json& { return j["entities"][1]["components"]; };

    // Knobs load.
    EXPECT_NO_THROW(load([&](nlohmann::json& j) {
        j["missile_doctrine"]["seeker"] = {{"reference_range_nmi", 12.0}, {"fov_half_angle_deg", 40.0}};
        j["missile_doctrine"]["datalink"] = {{"update_interval_s", 2.0}};
        pursuer(j)["airborne_radar"] = {{"reference_range_nmi", 40.0}};
        target(j)["signature"] = {{"rcs_scale", 2.0}};
        j["environment"] = {{"wind_speed_kts", 20.0}};
    }));
    // Type values do not.
    EXPECT_THROW(load([](nlohmann::json& j) { j["missile_doctrine"]["falloff_width_m"] = 30.0; }), std::runtime_error);
    EXPECT_THROW(load([](nlohmann::json& j) { j["missile_doctrine"]["thrust_profile"] = {{"boost_duration_s", 4.0}}; }),
                 std::runtime_error);
    EXPECT_THROW(load([](nlohmann::json& j) {
                     j["missile_doctrine"]["seeker"] = {{"active_seeker", {{"system_loss_db", 1.0}}}};
                 }),
                 std::runtime_error);
    EXPECT_THROW(load([&](nlohmann::json& j) { pursuer(j)["airborne_radar"] = {{"firing_threshold_db", 24.8}}; }),
                 std::runtime_error);
    EXPECT_THROW(load([&](nlohmann::json& j) { target(j)["evasion"]["rwr"] = {{"signal_jitter_db", 1.5}}; }),
                 std::runtime_error);
    // Nor do keys that are no parameter.
    EXPECT_THROW(load([](nlohmann::json& j) { j["missile_doctrine"]["nav_gian"] = 4.0; }), std::runtime_error);
    EXPECT_THROW(load([&](nlohmann::json& j) { pursuer(j)["airborne_radar"] = {{"no_such_key", 1.0}}; }),
                 std::runtime_error);
    EXPECT_THROW(load([](nlohmann::json& j) { j["environment"] = {{"humidity", 0.5}}; }), std::runtime_error);
}

// Every aircraft carries its type's radar signature, and only its type's:
// a scenario radar_cross_section block on an aircraft is a load error, and
// a signature block sets only the rcs_scale knob.
TEST(ScenarioSmoke, AircraftSignatureComesFromTheType) {
    nlohmann::json json = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    World world;
    ScenarioLoader::LoadFromJson(json, world);
    const auto& type = AircraftCatalog::Default().Get("f16c_block50")->signature;
    ASSERT_TRUE(type.present);
    int seen = 0;
    for (auto& [entity, tag] : world.Components<Tag>()) {
        const RadarCrossSection* rcs = world.Components<RadarCrossSection>().TryGet(entity);
        ASSERT_NE(rcs, nullptr) << tag.name;
        EXPECT_DOUBLE_EQ(rcs->staticRcsM2, type.staticRcsM2) << tag.name;
        EXPECT_EQ(rcs->fluctuation, type.fluctuation) << tag.name;
        ++seen;
    }
    EXPECT_EQ(seen, 2);

    // The one knob a scenario may set on it: rcs_scale.
    nlohmann::json scaled = json;
    for (nlohmann::json& entity : scaled["entities"]) {
        if (entity["name"] == "Target") entity["components"]["signature"] = {{"rcs_scale", 4.0}};
    }
    World scaledWorld;
    ScenarioLoader::LoadFromJson(scaled, scaledWorld);
    for (auto& [entity, tag] : scaledWorld.Components<Tag>()) {
        EXPECT_EQ(scaledWorld.Components<RadarCrossSection>().Get(entity).rcsScale, tag.name == "Target" ? 4.0 : 1.0)
            << tag.name;
    }

    for (nlohmann::json& entity : json["entities"]) {
        if (entity["name"] == "Target") entity["components"]["radar_cross_section"] = {{"static_rcs_m2", 2.0}};
    }
    World badWorld;
    EXPECT_THROW(ScenarioLoader::LoadFromJson(json, badWorld), std::runtime_error);
}

// The missile is pre-created at load, so a "(next launch)"
// edit made before the first launch must be copied into it
// (LaunchSystem::RefreshUnlaunched, which the parameter panel calls) or the
// shot flies the load-time doctrine. Launch mode is the visible case: a TWS
// scenario switched to STT must launch STT-gated.
TEST(ScenarioSmoke, NextLaunchEditBeforeFirstLaunchReachesTheMissile) {
    World world;
    SimContext simContext;
    ScenarioData scenario = ScenarioLoader::Load("tests/fixtures/scenarios/scenario_notch_beam.json", world);
    ASSERT_EQ(scenario.missileLaunchMode, LaunchMode::TWS);
    SimLoop::BeginRun(scenario, simContext, scenario.seed);
    ASSERT_TRUE(LaunchSystem::InitFromTags(world, scenario));

    scenario.missileLaunchMode = LaunchMode::STT;
    scenario.missileSelfDestructMaxTimeOfFlightS = 90.0;
    LaunchSystem::RefreshUnlaunched(world, scenario);

    std::optional<EntityID> missile = EntityLookup::FindByTagName(world, "Missile");
    ASSERT_TRUE(missile.has_value());
    GuidanceComponent& guidance = world.Components<GuidanceComponent>().Get(*missile);
    EXPECT_EQ(guidance.launchMode, LaunchMode::STT);
    EXPECT_DOUBLE_EQ(guidance.selfDestructMaxTimeOfFlightS, 90.0);

    std::string launchData;
    double simTime = 0.0;
    while (simTime < kMaxSimTime && launchData.empty()) {
        SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
        for (const auto& event : simContext.eventLog.Events())
            if (event.type == "LAUNCH") launchData = event.data;
    }
    ASSERT_FALSE(launchData.empty()) << "never launched";
    EXPECT_NE(launchData.find("gate=stt"), std::string::npos) << launchData;

    // Once fired, the missile keeps what it launched with.
    scenario.missileSelfDestructMaxTimeOfFlightS = 30.0;
    LaunchSystem::RefreshUnlaunched(world, scenario);
    EXPECT_DOUBLE_EQ(guidance.selfDestructMaxTimeOfFlightS, 90.0);
}

// Each rejection names the scenario's offending key.
TEST(ScenarioSmoke, LoaderErrorsNameTheKey) {
    using nlohmann::json;
    struct Case {
        const char* pointer;
        json value;  // null: erase the key
        const char* expected;
    };
    const json route = {{"points", {{0.0, 25000.0, 0.0}, {1000.0, 25000.0, 0.0}}}, {"loop", true}, {"loop_from", 2}};
    const json sensorWithoutAircraft = {{"name", "Box"},
                                        {"type", "aircraft"},
                                        {"transform", {{"pos", {0.0, 1000.0, 0.0}}, {"heading", 0.0}}},
                                        {"components", {{"airborne_radar", json::object()}}}};
    const std::vector<Case> cases = {
        {"/sim_speed", nullptr, "Scenario missing required field: sim_speed"},
        {"/missile_doctrine/launch_mode", "BOGUS", "/missile_doctrine/launch_mode must be one of"},
        {"/missile_doctrine/seeker/active_seeker/made_up_key", 100.0,
         "/missile_doctrine/seeker/active_seeker/made_up_key is not a scenario parameter"},
        {"/missile_doctrine/datalink/made_up_key", 10.0, "/missile_doctrine/datalink/made_up_key is not a scenario parameter"},
        {"/made_up_block", json::array(), "scenario has an unknown key \"made_up_block\""},
        {"/entities/0/components/airborne_radar/made_up_key", 1000.0,
         "airborne_radar/made_up_key is not a scenario parameter"},
        {"/entities/0/components/jsbsim/made_up_key", 0.5, "jsbsim has an unknown key \"made_up_key\""},
        {"/entities/1/components/radar_cross_section", {{"static_rcs_m2", 5.0}},
         "radar_cross_section is not accepted on an aircraft"},
        {"/entities/1/components/evasion", {{"rwr", {{"made_up_key", 1e-9}}}},
         "evasion.rwr has an unknown key \"made_up_key\""},
        {"/entities/1/components/made_up_block", json::object(), "components has an unknown key \"made_up_block\""},
        {"/entities/1/components/waypoints", route, "waypoints.loop_from must index one of the route's points"},
        {"/entities/2", sensorWithoutAircraft, "airborne_radar needs an aircraft"},
    };
    const json base = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    for (const Case& c : cases) {
        json scenario = base;
        json::json_pointer pointer(c.pointer);
        if (c.value.is_null()) {
            scenario[pointer.parent_pointer()].erase(pointer.back());
        } else {
            scenario[pointer] = c.value;
        }
        World world;
        try {
            ScenarioLoader::LoadFromJson(scenario, world);
            ADD_FAILURE() << c.pointer << ": expected a load error containing: " << c.expected;
        } catch (const std::runtime_error& e) {
            EXPECT_NE(std::string(e.what()).find(c.expected), std::string::npos) << c.pointer << ": " << e.what();
        }
    }
}

// Test rigs and fitting tools match a published gross weight by fuel
// alone: internal tanks first, then external, and never beyond either.
TEST(ScenarioSmoke, SetGrossWeightFillsInternalTanksFirst) {
    testrig::AircraftRig rig(testrig::kF16, 25000.0, 400.0, 0.0);
    std::shared_ptr<const AircraftType> type = AircraftCatalog::Default().Get(testrig::kF16);
    const FlightDynamicsSystem::FuelCapacity capacity = FlightDynamicsSystem::FuelCapacityOf(*type);
    ASSERT_GT(capacity.internalLbf, 0.0);
    ASSERT_GT(capacity.externalLbf, 0.0);
    auto tanks = [&rig, &type](bool internal) {
        double lbs = 0.0;
        auto propulsion = rig.Fdm().GetPropulsion();
        for (unsigned int i = 0; i < propulsion->GetNumTanks(); ++i) {
            auto tank = propulsion->GetTank(i);
            if ((tank->GetName().rfind(type->internalTankPrefix, 0) == 0) == internal) lbs += tank->GetContents();
        }
        return lbs;
    };

    double halfInternal = capacity.zeroFuelWeightLbf + 0.5 * capacity.internalLbf;
    FlightDynamicsSystem::SetGrossWeight(rig.Fdm(), *type, capacity, halfInternal);
    EXPECT_NEAR(tanks(true), 0.5 * capacity.internalLbf, 1.0);
    EXPECT_NEAR(tanks(false), 0.0, 1e-9);

    double withExternal = capacity.zeroFuelWeightLbf + capacity.internalLbf + 0.25 * capacity.externalLbf;
    FlightDynamicsSystem::SetGrossWeight(rig.Fdm(), *type, capacity, withExternal);
    EXPECT_NEAR(tanks(true), capacity.internalLbf, 1.0);
    EXPECT_NEAR(tanks(false), 0.25 * capacity.externalLbf, 1.0);
    rig.Fdm().Run();
    EXPECT_NEAR(rig.Prop("inertia/weight-lbs"), withExternal, 1.0);

    for (double impossible : {capacity.zeroFuelWeightLbf,
                              capacity.zeroFuelWeightLbf + capacity.internalLbf + capacity.externalLbf + 1.0}) {
        EXPECT_THROW(FlightDynamicsSystem::SetGrossWeight(rig.Fdm(), *type, capacity, impossible), std::runtime_error)
            << impossible;
    }
}
