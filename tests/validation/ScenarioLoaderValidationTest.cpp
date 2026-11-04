// What the scenario loader refuses, and what it says when it does. A typo, a
// stale key, a duplicate name or an impossible number must fail loudly naming
// the offender: a scenario that loads quietly into a different run than the
// file describes teaches the wrong thing. (The rejected-key messages and the
// teaching-knob rules are pinned in ScenarioSmokeValidationTest.)

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "components/RadarCrossSection.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"
#include "scenario/ScenarioLoaderJson.h"
#include "util/EntityLookup.h"

namespace {

using nlohmann::json;

struct Case {
    const char* pointer;
    json value;  // null: erase the key
    const char* expected;
};

// Loads the basic two-aircraft fixture with `c` applied and checks that the
// load fails with a message containing c.expected.
void ExpectRejected(const Case& c) {
    json scenario = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
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

}  // namespace

TEST(ScenarioLoaderRules, DuplicateEntityNamesAreRejected) {
    // Everything that finds an entity by name (the saver, the launcher's
    // Pursuer/Target lookup, the GUI) takes the first match, so a second
    // entity with the same name would be silently aliased onto the first.
    ExpectRejected({"/entities/1/name", "Pursuer", "used by more than one entity"});
}

TEST(ScenarioLoaderRules, UnknownKeysAreRejectedInEveryBlockTheLoaderReads) {
    const json route = {{"points", {{0.0, 25000.0, 0.0}}}, {"loopFrom", 0}};
    const std::vector<Case> cases = {
        {"/missle_doctrine", json::object(), "unknown key \"missle_doctrine\""},
        {"/entities/0/colour", "red", "unknown key \"colour\""},
        {"/entities/0/transform/alt", 1.0, "unknown key \"alt\""},
        {"/entities/0/components/airbone_radar", json::object(), "unknown key \"airbone_radar\""},
        {"/entities/0/components/jsbsim/altitude", 1.0, "unknown key \"altitude\""},
        {"/entities/1/components/waypoints", route, "unknown key \"loopFrom\""},
        {"/entities/1/components/velocity", {{"linear", {0.0, 0.0, 1.0}}, {"speed", 3.0}}, "unknown key \"speed\""},
    };
    for (const Case& c : cases) ExpectRejected(c);
}

TEST(ScenarioLoaderRules, UnknownKeysInAKinematicRadarCrossSectionAreRejected) {
    json scenario = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    scenario["entities"].push_back(
        {{"name", "Drone"},
         {"type", "aircraft"},
         {"transform", {{"pos", {0.0, 1000.0, 0.0}}, {"heading", 0.0}}},
         {"components",
          {{"velocity", {{"linear", {0.0, 0.0, 10.0}}}},
           {"radar_cross_section",
            {{"static_rcs_m2", 2.0}, {"fluctuation_model", "steady"}, {"fluctuation_dbm", 1.0}}}}}});
    World world;
    try {
        ScenarioLoader::LoadFromJson(scenario, world);
        ADD_FAILURE() << "expected an unknown-key error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("unknown key \"fluctuation_dbm\""), std::string::npos) << e.what();
    }
}

TEST(ScenarioLoaderRules, ImpossibleNumbersAreRejected) {
    const json oneAndAHalf = 1.5;
    const std::vector<Case> cases = {
        {"/seed", -1, "seed must be a non-negative integer"},
        {"/seed", oneAndAHalf, "seed must be a non-negative integer"},
        {"/seed", "abc", "seed must be a non-negative integer"},
        // Zero or negative speed means the clock never advances.
        {"/sim_speed", 0.0, "sim_speed must be greater than 0"},
        {"/sim_speed", -2.0, "sim_speed must be greater than 0"},
        // A route whose points can never be reached.
        {"/entities/1/components/waypoints", {{"points", {{0.0, 25000.0, 0.0}}}, {"arrival_radius_m", 0.0}},
         "arrival_radius_m must be greater than 0"},
        {"/entities/1/components/waypoints", {{"points", {{0.0, 25000.0, 0.0}}}, {"arrival_radius_m", -5.0}},
         "arrival_radius_m must be greater than 0"},
    };
    for (const Case& c : cases) ExpectRejected(c);
}

// The registry's range is what the GUI lets a user set. A file or a batch
// override outside it is a value nobody meant: a negative nav gain, a wind
// that silently reverses, a reference range of zero that the radar equation
// quietly ignores while the results table labels the run "0".
TEST(ScenarioLoaderRules, KnobsOutsideTheirRegistryRangeAreRejected) {
    const std::vector<Case> cases = {
        {"/missile_doctrine/nav_gain", -3.0, "nav_gain"},
        {"/missile_doctrine/kill_radius_m", -1.0, "kill_radius_m"},
        {"/missile_doctrine/launch_range_fraction", 0.0, "launch_range_fraction"},
        {"/missile_doctrine/launch_range_fraction", 1.5, "launch_range_fraction"},
        {"/environment", {{"wind_speed_kts", -50.0}}, "wind_speed_kts"},
        {"/environment", {{"wind_speed_kts", 400.0}}, "wind_speed_kts"},
        {"/environment", {{"temperature_offset_c", 80.0}}, "temperature_offset_c"},
        {"/entities/0/components/airborne_radar", {{"reference_range_nmi", 0.0}}, "reference_range_nmi"},
        {"/entities/1/components/signature", {{"rcs_scale", 0.0}}, "rcs_scale"},
    };
    for (const Case& c : cases) ExpectRejected(c);
}

TEST(ScenarioLoaderRules, KnobsAtTheEdgesOfTheirRangeLoad) {
    for (double radius : {0.0, 200.0}) {
        json scenario = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
        scenario["missile_doctrine"]["kill_radius_m"] = radius;
        scenario["environment"] = {{"wind_speed_kts", radius == 0.0 ? 0.0 : 300.0},
                                   {"wind_from_deg", radius == 0.0 ? 0.0 : 360.0}};
        World world;
        EXPECT_NO_THROW(ScenarioLoader::LoadFromJson(scenario, world)) << radius;
    }
}

TEST(ScenarioLoaderRules, AKinematicTargetsCrossSectionCannotBeNegative) {
    json scenario = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    scenario["entities"].push_back(
        {{"name", "Drone"},
         {"type", "aircraft"},
         {"transform", {{"pos", {0.0, 1000.0, 0.0}}, {"heading", 0.0}}},
         {"components",
          {{"velocity", {{"linear", {0.0, 0.0, 10.0}}}},
           {"radar_cross_section", {{"static_rcs_m2", -1.0}, {"fluctuation_model", "steady"}}}}}});
    World world;
    try {
        ScenarioLoader::LoadFromJson(scenario, world);
        ADD_FAILURE() << "expected a static_rcs_m2 error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("static_rcs_m2 must be"), std::string::npos) << e.what();
    }
}

// An aircraft states its start altitude and heading twice, in "transform"
// and in "jsbsim". The flight model starts from jsbsim.*; the transform is
// what the saver and a placement edit change. Allowed to disagree, a file
// would start the aircraft somewhere it does not say.
TEST(ScenarioLoaderRules, AircraftStartAltitudeAndHeadingMustBeStatedConsistently) {
    ExpectRejected({"/entities/0/transform/pos", {-150000.0, 35000.0, 0.0}, "jsbsim.altitude_ft"});
    ExpectRejected({"/entities/0/transform/heading", 45.0, "jsbsim.heading_deg"});
}

TEST(ScenarioLoaderRules, HeadingsAreComparedAroundTheCircle) {
    json scenario = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    scenario["entities"][0]["transform"]["heading"] = 360.0;
    scenario["entities"][0]["components"]["jsbsim"]["heading_deg"] = 0.0;
    World world;
    EXPECT_NO_THROW(ScenarioLoader::LoadFromJson(scenario, world));
}

namespace {

json BasicWithDrone(const json& droneComponents) {
    json scenario = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    scenario["entities"].push_back({{"name", "Drone"},
                                    {"type", "aircraft"},
                                    {"transform", {{"pos", {0.0, 1000.0, 0.0}}, {"heading", 0.0}}},
                                    {"components", droneComponents}});
    return scenario;
}

}  // namespace

// The GUI shows an RCS-scale row for any entity with a cross-section and the
// saver writes it back as components.signature.rcs_scale, so a kinematic
// target's scale must load: dropping it would quietly reset the knob.
TEST(ScenarioLoaderRules, AKinematicTargetsRcsScaleLoads) {
    json scenario = BasicWithDrone({{"velocity", {{"linear", {0.0, 0.0, 10.0}}}},
                                    {"radar_cross_section", {{"static_rcs_m2", 4.0}, {"fluctuation_model", "steady"}}},
                                    {"signature", {{"rcs_scale", 3.0}}}});
    World world;
    ScenarioLoader::LoadFromJson(scenario, world);
    std::optional<EntityID> drone = EntityLookup::FindByTagName(world, "Drone");
    ASSERT_TRUE(drone.has_value());
    EXPECT_EQ(world.Components<RadarCrossSection>().Get(*drone).rcsScale, 3.0);
    EXPECT_EQ(world.Components<RadarCrossSection>().Get(*drone).staticRcsM2, 4.0);
}

TEST(ScenarioLoaderRules, ASignatureOnAnEntityWithNoCrossSectionIsRejected) {
    json scenario = BasicWithDrone({{"velocity", {{"linear", {0.0, 0.0, 10.0}}}},
                                    {"signature", {{"rcs_scale", 3.0}}}});
    World world;
    try {
        ScenarioLoader::LoadFromJson(scenario, world);
        ADD_FAILURE() << "expected a signature error";
    } catch (const std::runtime_error& e) {
        EXPECT_NE(std::string(e.what()).find("signature needs"), std::string::npos) << e.what();
    }
}

// A bare "array index out of range" tells the author nothing about which of
// fifty entities to look at.
TEST(ScenarioLoaderRules, MalformedEntityValuesNameTheEntity) {
    const std::vector<Case> cases = {
        {"/entities/0/transform/pos", {1.0, 2.0}, "entity 'Pursuer'"},
        {"/entities/1/transform/heading", "north", "entity 'Target'"},
        {"/entities/1/components/jsbsim/altitude_ft", "high", "entity 'Target'"},
    };
    for (const Case& c : cases) ExpectRejected(c);
}
