// The saver is load-bearing for the whole tuning workflow: edits made in
// the viewer reach a Monte Carlo sweep only by being written to a
// scenario file, and Reset rebuilds the world from that same JSON. So the
// checks here are less about "does saving work" than about the specific
// ways a silently wrong sweep baseline could be produced -- asymmetric
// unit conversion, a mistyped JSON pointer, enum round-trip error, and a
// mid-run save capturing live state instead of the authored start --
// plus the guarantee that an unedited Reset reproduces a run exactly.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include "JsbSimDebugSilencer.h"
#include "components/GuidanceComponent.h"
#include "components/SpawnPose.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "components/WaypointComponent.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "scenario/ParamRegistry.h"
#include "scenario/ScenarioLoader.h"
#include "scenario/ScenarioLoaderJson.h"
#include "scenario/ScenarioSaver.h"
#include "scenario/ScenarioSaverJson.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"
#include "sim/SimLoop.h"
#include "systems/LaunchSystem.h"
#include "systems/WaypointSystem.h"
#include "util/EntityLookup.h"

namespace {

using EntityLookup::FindByTagName;

constexpr const char* kSourcePath = "tests/fixtures/scenarios/scenario_waypoint_patrol.json";

// Saves go to the test framework's temp directory, one file per test:
// ctest -j runs each TEST() as its own process, so a shared path would let
// one test read or delete another's save, and a failed assertion before
// cleanup leaves nothing behind in the repo.
std::string ScratchPath() {
    return ::testing::TempDir() + "scenario_saver_scratch_" +
           ::testing::UnitTest::GetInstance()->current_test_info()->name() + ".json";
}

bool NearlyEqual(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

// Relative, for the parameters spanning twenty orders of magnitude
// (detection thresholds at 1e-16 alongside weights at 1e5).
bool NearlyEqualRel(double a, double b) {
    double scale = std::max({1.0, std::fabs(a), std::fabs(b)});
    return std::fabs(a - b) <= 1e-9 * scale;
}

// A value inside the parameter's own range that is definitely different
// from `live`. Seven drag steps is far enough to survive a float round
// trip through the JSON and small enough to keep a scenario loadable.
double PerturbedValue(const ParamRegistry::ParamDef& param, double live) {
    double delta = param.dragStep * 7.0;
    if (delta == 0.0) delta = std::max(1.0, std::fabs(live) * 0.1);
    if (live + delta <= param.maxValue) return live + delta;
    if (live - delta >= param.minValue) return live - delta;
    return (param.minValue + param.maxValue) * 0.5;
}

// --- Structural checks on the registry table itself. Cheap, and catches
// the copy-paste mistakes a table this long invites.
TEST(ScenarioSaver, RegistryIntegrity) {
    const std::vector<ParamRegistry::ParamDef>& all = ParamRegistry::All();
    int badAccessor = 0;
    int badScale = 0;
    std::vector<std::string> duplicates;

    for (const ParamRegistry::ParamDef& param : all) {
        int accessors = (param.dbl ? 1 : 0) + (param.flag ? 1 : 0) + (param.enumAccess.get ? 1 : 0);
        bool matches = (param.type == ParamRegistry::ParamType::Double && param.dbl) ||
                       (param.type == ParamRegistry::ParamType::Bool && param.flag) ||
                       (param.type == ParamRegistry::ParamType::Enum && param.enumAccess.get);
        if (accessors != 1 || !matches) ++badAccessor;
        // A zero or negative conversion factor would divide to infinity on
        // save -- the failure mode would be a corrupt file, not a wrong one.
        if (!(param.jsonScale.factor > 0.0)) ++badScale;
    }

    for (size_t i = 0; i < all.size(); ++i) {
        for (size_t k = i + 1; k < all.size(); ++k) {
            if (std::string(all[i].pointer) == all[k].pointer) duplicates.push_back(all[i].pointer);
        }
    }

    EXPECT_EQ(badAccessor, 0);
    EXPECT_EQ(badScale, 0);
    EXPECT_TRUE(duplicates.empty()) << "first duplicate: " << (duplicates.empty() ? "" : duplicates[0]);
}

// --- Every registry parameter survives save/reload. This is the check
// the tuning workflow rests on. Perturb everything the registry can
// reach, save, reload into a fresh World, and confirm each value came
// back. A unit conversion applied asymmetrically, or a pointer that does
// not match the loader's key, shows up here and nowhere else.
void RunParamRoundTrip(const char* scenarioPath) {
    World world;
    ScenarioData scenario;
    ASSERT_NO_THROW({
        scenario = ScenarioLoader::Load(scenarioPath, world);
    }) << "failed to load "
       << scenarioPath;

    std::ifstream sourceFile(scenarioPath);
    nlohmann::json sourceJson;
    sourceFile >> sourceJson;

    // The saver refuses to create a missile_doctrine block from a single
    // edited key, because the loader requires three core fields and a
    // partial block would be unloadable. Mirror that here rather than
    // expecting doctrine to survive in a scenario that has none.
    bool hasDoctrine = sourceJson.contains("missile_doctrine");

    // Subjects are matched by tag name, the same way the saver resolves
    // them back to array indices.
    std::vector<std::pair<std::string, EntityID>> entitySubjects;
    if (sourceJson.contains("entities")) {
        for (const nlohmann::json& element : sourceJson.at("entities")) {
            std::string name = element.at("name").get<std::string>();
            std::optional<EntityID> entity = FindByTagName(world, name);
            if (entity.has_value()) entitySubjects.emplace_back(name, *entity);
        }
    }

    struct Expectation {
        const ParamRegistry::ParamDef* param;
        std::string subjectName;
        double dbl = 0.0;
        bool flag = false;
        int enumIndex = 0;
    };
    std::vector<Expectation> expectations;

    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        // Only a teaching knob is editable, so only a knob is saved.
        if (!param.knob) continue;
        // Doctrine applies once; entity params apply to every entity that
        // actually carries the component.
        std::vector<std::pair<std::string, EntityID>> targets;
        switch (param.owner) {
            case ParamRegistry::ParamOwner::Doctrine:
                if (!hasDoctrine) continue;
                targets.emplace_back("", EntityID{0});
                break;
            case ParamRegistry::ParamOwner::Environment:
                targets.emplace_back("", EntityID{0});
                break;
            case ParamRegistry::ParamOwner::Entity:
                targets = entitySubjects;
                break;
        }

        for (const auto& [name, entity] : targets) {
            Expectation expectation;
            expectation.param = &param;
            expectation.subjectName = name;
            switch (param.type) {
                case ParamRegistry::ParamType::Double: {
                    double* live = param.dbl(scenario, world, entity);
                    if (!live) continue;
                    *live = PerturbedValue(param, *live);
                    expectation.dbl = *live;
                    break;
                }
                case ParamRegistry::ParamType::Bool: {
                    bool* live = param.flag(scenario, world, entity);
                    if (!live) continue;
                    *live = !*live;
                    expectation.flag = *live;
                    break;
                }
                case ParamRegistry::ParamType::Enum: {
                    int live = param.enumAccess.get(scenario, world, entity);
                    if (live < 0) continue;
                    int next = (live + 1) % static_cast<int>(param.enumAccess.names.size());
                    param.enumAccess.set(scenario, world, entity, next);
                    expectation.enumIndex = next;
                    break;
                }
            }
            expectations.push_back(expectation);
        }
    }

    std::string saveError;
    ASSERT_TRUE(ScenarioSaver::Save(scenarioPath, ScratchPath(), world, scenario, saveError)) << saveError;

    World reloaded;
    ScenarioData reloadedScenario;
    try {
        reloadedScenario = ScenarioLoader::Load(ScratchPath(), reloaded);
    } catch (const std::exception& e) {
        std::filesystem::remove(ScratchPath());
        FAIL() << "failed to reload saved file: " << e.what();
    }

    int mismatches = 0;
    std::string firstMismatch;
    for (const Expectation& expectation : expectations) {
        const ParamRegistry::ParamDef& param = *expectation.param;
        EntityID entity = 0;
        if (!expectation.subjectName.empty()) {
            std::optional<EntityID> found = FindByTagName(reloaded, expectation.subjectName);
            if (!found.has_value()) {
                ++mismatches;
                continue;
            }
            entity = *found;
        }

        bool ok = false;
        switch (param.type) {
            case ParamRegistry::ParamType::Double: {
                double* value = param.dbl(reloadedScenario, reloaded, entity);
                ok = value && NearlyEqualRel(*value, expectation.dbl);
                break;
            }
            case ParamRegistry::ParamType::Bool: {
                bool* value = param.flag(reloadedScenario, reloaded, entity);
                ok = value && *value == expectation.flag;
                break;
            }
            case ParamRegistry::ParamType::Enum:
                ok = param.enumAccess.get(reloadedScenario, reloaded, entity) == expectation.enumIndex;
                break;
        }
        if (!ok) {
            if (firstMismatch.empty()) {
                firstMismatch = std::string(param.pointer) + " (subject \"" + expectation.subjectName + "\")";
            }
            ++mismatches;
        }
    }

    std::filesystem::remove(ScratchPath());
    EXPECT_EQ(mismatches, 0) << expectations.size() << " parameter instance(s) perturbed, first mismatch: "
                              << firstMismatch;
}

TEST(ScenarioSaver, ParamRoundTripScenarioBasic) {
    // Covers missile doctrine and the aircraft component set.
    RunParamRoundTrip("tests/fixtures/scenarios/scenario_basic.json");
}

TEST(ScenarioSaver, ParamRoundTripScenarioNotchBeam) {
    // Covers the RWR block, which lives under components.evasion.
    RunParamRoundTrip("tests/fixtures/scenarios/scenario_notch_beam.json");
}

TEST(ScenarioSaver, ParamRoundTripScenarioWithAKinematicTarget) {
    // "TargetInFOV" has a radar_cross_section and a velocity but no jsbsim
    // block: its RCS scale is editable in the GUI and must survive a save.
    RunParamRoundTrip("tests/fixtures/scenarios/scenario_airborne_radar_test.json");
}

// --- An untouched load/save adds nothing. What makes "write only what
// changed" a real property rather than an aspiration: with no edits, the
// output must be the source.
void RunUnchangedFidelity(const char* scenarioPath) {
    World world;
    ScenarioData scenario;
    ASSERT_NO_THROW({
        scenario = ScenarioLoader::Load(scenarioPath, world);
    }) << "failed to load "
       << scenarioPath;

    nlohmann::json built;
    std::string error;
    ASSERT_TRUE(ScenarioSaver::BuildJson(scenarioPath, world, scenario, built, error)) << error;

    std::ifstream sourceFile(scenarioPath);
    nlohmann::json source;
    sourceFile >> source;

    // Waypoint blocks are always rewritten (the saver regenerates them
    // from the live queue), so compare everything else. A spurious key
    // anywhere outside waypoints means a baseline comparison is wrong.
    nlohmann::json strippedBuilt = built;
    nlohmann::json strippedSource = source;
    for (nlohmann::json* doc : {&strippedBuilt, &strippedSource}) {
        if (!doc->contains("entities")) continue;
        for (auto& entityJson : doc->at("entities")) {
            if (entityJson.contains("components")) entityJson["components"].erase("waypoints");
        }
    }

    EXPECT_EQ(strippedBuilt, strippedSource);
}

TEST(ScenarioSaver, UnchangedFidelityScenarioBasic) {
    RunUnchangedFidelity("tests/fixtures/scenarios/scenario_basic.json");
}

TEST(ScenarioSaver, UnchangedFidelityScenarioNotchBeam) {
    RunUnchangedFidelity("tests/fixtures/scenarios/scenario_notch_beam.json");
}

// --- Saving mid-run must write each entity's authored start, not the
// position it has since flown to -- otherwise Reset after a run moves
// the start line.
TEST(ScenarioSaver, MidRunSaveKeepsSpawnPose) {
    constexpr const char* kScenario = "tests/fixtures/scenarios/scenario_basic.json";
    World world;
    SimContext simContext;
    ScenarioData scenario;
    ASSERT_NO_THROW({
        scenario = ScenarioLoader::Load(kScenario, world);
    });
    SimLoop::BeginRun(scenario, simContext, scenario.seed);

    std::ifstream sourceFile(kScenario);
    nlohmann::json source;
    sourceFile >> source;

    double simTime = 0.0;
    for (int i = 0; i < 1200; ++i) SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);

    nlohmann::json built;
    std::string error;
    ASSERT_TRUE(ScenarioSaver::BuildJson(kScenario, world, scenario, built, error)) << error;
    for (size_t i = 0; i < source.at("entities").size(); ++i) {
        EXPECT_EQ(built["entities"][i]["transform"], source["entities"][i]["transform"]) << "entity " << i;
    }
}

// --- A placement edit (the GUI's Placement rows) followed by Reset must
// start the aircraft there: altitude and heading as well as east/north.
// The flight model takes its start altitude and heading from the jsbsim
// block, so the saver keeps that block in step with the placement.
TEST(ScenarioSaver, PlacementEditReachesTheFlightModelOnReset) {
    constexpr const char* kScenario = "tests/fixtures/scenarios/scenario_basic.json";
    World world;
    ScenarioData scenario = ScenarioLoader::Load(kScenario, world);
    EntityID pursuer = EntityLookup::FindByTagName(world, "Pursuer").value();
    SpawnPose& spawn = world.Components<SpawnPose>().Get(pursuer);
    spawn.pos.x() -= 20000.0;
    spawn.pos.y() = 35000.0;
    spawn.headingDeg = 60.0;

    std::string error;
    ASSERT_TRUE(ScenarioSaver::ResetWorld(kScenario, world, scenario, error)) << error;
    SimContext simContext;
    SimLoop::BeginRun(scenario, simContext, scenario.seed);
    double simTime = 0.0;
    for (int i = 0; i < 120; ++i) SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);

    pursuer = EntityLookup::FindByTagName(world, "Pursuer").value();
    const Transform& transform = world.Components<Transform>().Get(pursuer);
    EXPECT_NEAR(transform.pos.y(), 35000.0, 300.0);
    EXPECT_NEAR(transform.euler.z(), 60.0, 5.0);
    EXPECT_NEAR(transform.pos.x(), -170000.0, 1500.0);  // ~1 s of flight east of the new start
}

// Runs a compound scenario to resolution, returning the resolution time
// (or the timeout). Mirrors what the batch runner does.
double RunToResolution(World& world, SimContext& simContext, ScenarioData& scenario) {
    LaunchSystem::InitFromTags(world, scenario);

    double simTime = 0.0;
    constexpr double kMaxSimTime = 400.0;
    while (simTime < kMaxSimTime && !simContext.outcome.resolved) {
        SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
    }
    return simTime;
}

// --- Reset with no edits reproduces the run. The property that makes
// Reset useful for A/B-ing one parameter: if an untouched reset changed
// the outcome, you could never tell your change from the reset itself.
// Also proves BuildJson is lossless, which is the same guarantee Save As
// depends on.
TEST(ScenarioSaver, ResetDeterminism) {
    constexpr const char* kScenario = "tests/fixtures/scenarios/scenario_basic.json";

    World worldA;
    SimContext contextA;
    ScenarioData scenarioA;
    ASSERT_NO_THROW({
        scenarioA = ScenarioLoader::Load(kScenario, worldA);
    }) << "failed to load "
       << kScenario;
    SimLoop::BeginRun(scenarioA, contextA, scenarioA.seed);
    double directTime = RunToResolution(worldA, contextA, scenarioA);
    bool directHit = contextA.outcome.hit;

    // A fresh load, then Reset before running -- the sequence a user gets
    // by loading a scenario and pressing Reset without touching anything.
    World worldB;
    SimContext contextB;
    ScenarioData scenarioB;
    ASSERT_NO_THROW({
        scenarioB = ScenarioLoader::Load(kScenario, worldB);
    }) << "failed to load "
       << kScenario;
    std::string resetError;
    ASSERT_TRUE(ScenarioSaver::ResetWorld(kScenario, worldB, scenarioB, resetError)) << resetError;
    SimLoop::BeginRun(scenarioB, contextB, scenarioB.seed);
    double resetTime = RunToResolution(worldB, contextB, scenarioB);
    bool resetHit = contextB.outcome.hit;

    EXPECT_TRUE(NearlyEqual(directTime, resetTime, 1e-9))
        << "direct run resolved at t=" << directTime << "s, after an unedited Reset t=" << resetTime << "s";
    EXPECT_EQ(directHit, resetHit);
}

// --- Reset keeps edits, discards the run.
TEST(ScenarioSaver, ResetPreservesEdits) {
    constexpr const char* kScenario = "tests/fixtures/scenarios/scenario_basic.json";

    World world;
    SimContext simContext;
    ScenarioData scenario;
    ASSERT_NO_THROW({
        scenario = ScenarioLoader::Load(kScenario, world);
    }) << "failed to load "
       << kScenario;
    SimLoop::BeginRun(scenario, simContext, scenario.seed);

    // An edit of each kind the GUI can make: a doctrine parameter, an
    // entity parameter, and a waypoint.
    scenario.missileDatalink.updateIntervalS = 7.25;
    std::optional<EntityID> target = FindByTagName(world, "Target");
    ASSERT_TRUE(target.has_value()) << "no Target entity";
    ComponentStore<WaypointComponent>& waypoints = world.Components<WaypointComponent>();
    if (!waypoints.Has(*target)) waypoints.Add(*target, WaypointComponent{});
    waypoints.Get(*target).queue.push_back(WaypointPoint{Vec3{4321.0, 26500.0, 8765.0}, 333.0});
    size_t expectedPoints = waypoints.Get(*target).queue.size();

    // Run until the missile is actually in flight, so the check that run
    // state is discarded means something.
    std::optional<EntityID> pursuer = FindByTagName(world, "Pursuer");
    ASSERT_TRUE(pursuer.has_value()) << "no Pursuer entity";
    LaunchSystem::Init(world, *pursuer, *target, scenario);
    double simTime = 0.0;
    while (!simContext.outcome.launched && simTime < 400.0) {
        SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
    }
    ASSERT_TRUE(simContext.outcome.launched) << "missile never launched";
    ASSERT_TRUE(FindByTagName(world, "Missile").has_value());

    std::string resetError;
    ASSERT_TRUE(ScenarioSaver::ResetWorld(kScenario, world, scenario, resetError)) << resetError;

    EXPECT_TRUE(NearlyEqual(scenario.missileDatalink.updateIntervalS, 7.25));
    std::optional<EntityID> resetTarget = FindByTagName(world, "Target");
    ComponentStore<WaypointComponent>& resetWaypoints = world.Components<WaypointComponent>();
    bool waypointKept = resetTarget.has_value() && resetWaypoints.Has(*resetTarget) &&
                        resetWaypoints.Get(*resetTarget).queue.size() == expectedPoints;
    EXPECT_TRUE(waypointKept);
    // The missile is gone: Reset rebuilt the world from the scenario, and
    // a missile only exists once LaunchSystem::Init runs.
    EXPECT_FALSE(FindByTagName(world, "Missile").has_value());
}

// --- Round-trip check: load a real scenario, mutate its Target's
// waypoint queue (add a point, edit a position, edit a speed override,
// remove a point), save, reload into a fresh World, and confirm every
// field of the reloaded queue matches what was saved -- exactly the
// workflow the waypoint edit panel + "Save As" button drive.
TEST(ScenarioSaver, SaveLoadRoundTripWaypoints) {
    World world;
    ScenarioData scenario;
    ASSERT_NO_THROW({
        scenario = ScenarioLoader::Load(kSourcePath, world);
    }) << "failed to load "
       << kSourcePath;

    std::optional<EntityID> patrol = FindByTagName(world, "Patrol");
    ASSERT_TRUE(patrol.has_value()) << kSourcePath << " missing expected \"Patrol\" tag";

    ComponentStore<WaypointComponent>& waypointStore = world.Components<WaypointComponent>();
    ASSERT_TRUE(waypointStore.Has(*patrol)) << "Patrol has no WaypointComponent";
    WaypointComponent& waypoint = waypointStore.Get(*patrol);

    // Mutate: edit an existing point's position (matches dragging in the
    // 3D view), add a speed override to a previously-unset point (matches
    // the edit panel's checkbox), and append a brand new point (matches
    // click-placement) -- exercising every kind of edit the GUI can make.
    ASSERT_FALSE(waypoint.queue.empty()) << "source scenario's waypoint queue is empty, nothing to mutate";
    waypoint.queue[0].pos = Vec3{12345.0, 26000.0, -6789.0};
    waypoint.queue[0].speedKts = 275.5;
    waypoint.queue.push_back(WaypointPoint{Vec3{999.0, 25500.0, -4321.0}, std::nullopt});
    bool savedLoop = waypoint.loop;
    double savedArrivalRadiusM = waypoint.arrivalRadiusM;
    size_t savedCount = waypoint.queue.size();

    // Copy the mutated queue out by value before Save/reload touch a
    // *different* World -- comparing against this copy, not the live
    // component, keeps the check honest regardless of what Save does
    // internally.
    std::vector<WaypointPoint> expected(waypoint.queue.begin(), waypoint.queue.end());

    std::string saveError;
    ASSERT_TRUE(ScenarioSaver::Save(kSourcePath, ScratchPath(), world, scenario, saveError)) << saveError;

    World reloadedWorld;
    ScenarioData reloadedScenario;
    try {
        reloadedScenario = ScenarioLoader::Load(ScratchPath(), reloadedWorld);
    } catch (const std::exception& e) {
        std::filesystem::remove(ScratchPath());
        FAIL() << "failed to reload " << ScratchPath() << ": " << e.what();
    }
    (void)reloadedScenario;

    std::optional<EntityID> reloadedPatrol = FindByTagName(reloadedWorld, "Patrol");
    ComponentStore<WaypointComponent>& reloadedWaypointStore = reloadedWorld.Components<WaypointComponent>();
    ASSERT_TRUE(reloadedPatrol.has_value() && reloadedWaypointStore.Has(*reloadedPatrol));

    const WaypointComponent& reloaded = reloadedWaypointStore.Get(*reloadedPatrol);
    EXPECT_EQ(reloaded.queue.size(), savedCount);
    EXPECT_EQ(reloaded.loop, savedLoop);
    EXPECT_TRUE(NearlyEqual(reloaded.arrivalRadiusM, savedArrivalRadiusM));
    for (size_t i = 0; i < expected.size() && i < reloaded.queue.size(); ++i) {
        EXPECT_TRUE(NearlyEqual(reloaded.queue[i].pos.x(), expected[i].pos.x())) << "point " << i;
        EXPECT_TRUE(NearlyEqual(reloaded.queue[i].pos.y(), expected[i].pos.y())) << "point " << i;
        EXPECT_TRUE(NearlyEqual(reloaded.queue[i].pos.z(), expected[i].pos.z())) << "point " << i;
        EXPECT_EQ(reloaded.queue[i].speedKts.has_value(), expected[i].speedKts.has_value()) << "point " << i;
        if (expected[i].speedKts.has_value() && reloaded.queue[i].speedKts.has_value()) {
            EXPECT_TRUE(NearlyEqual(*reloaded.queue[i].speedKts, *expected[i].speedKts)) << "point " << i;
        }
    }

    // Confirm a component this feature never touches survived byte-for-
    // byte -- the whole point of ScenarioSaver patching rather than
    // re-serializing the entity from scratch.
    EXPECT_TRUE(reloadedWorld.Components<Transform>().Has(*reloadedPatrol));

    std::filesystem::remove(ScratchPath());
}

// --- Waypoints the aircraft has already flown through must still be
// saved, in their authored order. WaypointSystem consumes the queue as
// points are reached (popping them, or rotating a looping route), so
// saving the live queue alone would drop or reorder the route. Drives the
// real WaypointSystem by placing the aircraft on each point in turn.
void RunPassedWaypointsSurviveSave(bool loop, size_t loopFrom = 0) {
    World world;
    ScenarioData scenario;
    ASSERT_NO_THROW({
        scenario = ScenarioLoader::Load(kSourcePath, world);
    });
    std::optional<EntityID> patrol = FindByTagName(world, "Patrol");
    ASSERT_TRUE(patrol.has_value());
    WaypointComponent& waypoint = world.Components<WaypointComponent>().Get(*patrol);
    waypoint.loop = loop;
    waypoint.loopFrom = loopFrom;
    std::vector<WaypointPoint> authored(waypoint.queue.begin(), waypoint.queue.end());
    ASSERT_GE(authored.size(), 3u);

    for (int reached = 0; reached < 2; ++reached) {
        world.Components<Transform>().Get(*patrol).pos = waypoint.queue.front().pos;
        WaypointSystem::Update(world, scenario.environment);
    }
    // Reached lead-in points (and every point of a non-looping route) leave
    // the queue; the repeating part rotates.
    size_t dropped = loop ? std::min<size_t>(loopFrom, 2) : 2;
    ASSERT_EQ(waypoint.queue.size(), authored.size() - dropped) << "WaypointSystem did not consume the reached points";

    std::string saveError;
    ASSERT_TRUE(ScenarioSaver::Save(kSourcePath, ScratchPath(), world, scenario, saveError)) << saveError;

    World reloadedWorld;
    try {
        ScenarioLoader::Load(ScratchPath(), reloadedWorld);
    } catch (const std::exception& e) {
        std::filesystem::remove(ScratchPath());
        FAIL() << "failed to reload " << ScratchPath() << ": " << e.what();
    }
    std::filesystem::remove(ScratchPath());

    std::optional<EntityID> reloadedPatrol = FindByTagName(reloadedWorld, "Patrol");
    ASSERT_TRUE(reloadedPatrol.has_value());
    const WaypointComponent& reloaded = reloadedWorld.Components<WaypointComponent>().Get(*reloadedPatrol);
    EXPECT_EQ(reloaded.loopFrom, loopFrom);
    ASSERT_EQ(reloaded.queue.size(), authored.size());
    for (size_t i = 0; i < authored.size(); ++i) {
        EXPECT_TRUE(NearlyEqual(reloaded.queue[i].pos.x(), authored[i].pos.x())) << "point " << i;
        EXPECT_TRUE(NearlyEqual(reloaded.queue[i].pos.y(), authored[i].pos.y())) << "point " << i;
        EXPECT_TRUE(NearlyEqual(reloaded.queue[i].pos.z(), authored[i].pos.z())) << "point " << i;
        EXPECT_EQ(reloaded.queue[i].speedKts.has_value(), authored[i].speedKts.has_value()) << "point " << i;
    }
}

TEST(ScenarioSaver, PassedWaypointsSurviveSave) { RunPassedWaypointsSurviveSave(/*loop=*/false); }

TEST(ScenarioSaver, LoopedRouteSavesInAuthoredOrder) { RunPassedWaypointsSurviveSave(/*loop=*/true); }

TEST(ScenarioSaver, PartlyLoopedRouteSavesInAuthoredOrder) {
    RunPassedWaypointsSurviveSave(/*loop=*/true, /*loopFrom=*/1);
}

// --- A route edited in the GUI after a loop has rotated. The GUI draws and
// edits the live queue (C A B once A is reached), so what a save must write
// is that queue as shown. Un-rotating by the old rotation count after the
// queue changed length would scramble it: C A B plus a new point saved as
// the new point first.
namespace {

void ExpectEditedRotatedLoopSavesAsShown(const std::function<void(WaypointComponent&)>& edit) {
    World world;
    ScenarioData scenario;
    ASSERT_NO_THROW({ scenario = ScenarioLoader::Load(kSourcePath, world); });
    std::optional<EntityID> patrol = FindByTagName(world, "Patrol");
    ASSERT_TRUE(patrol.has_value());
    WaypointComponent& waypoint = world.Components<WaypointComponent>().Get(*patrol);
    waypoint.loop = true;
    waypoint.loopFrom = 0;

    world.Components<Transform>().Get(*patrol).pos = waypoint.queue.front().pos;
    WaypointSystem::Update(world, scenario.environment);  // reaches the first point: the loop rotates once
    ASSERT_EQ(waypoint.loopRotation, 1u);

    edit(waypoint);
    std::vector<WaypointPoint> shown(waypoint.queue.begin(), waypoint.queue.end());

    std::string saveError;
    ASSERT_TRUE(ScenarioSaver::Save(kSourcePath, ScratchPath(), world, scenario, saveError)) << saveError;
    World reloadedWorld;
    try {
        ScenarioLoader::Load(ScratchPath(), reloadedWorld);
    } catch (const std::exception& e) {
        std::filesystem::remove(ScratchPath());
        FAIL() << "failed to reload " << ScratchPath() << ": " << e.what();
    }
    std::filesystem::remove(ScratchPath());

    std::optional<EntityID> reloadedPatrol = FindByTagName(reloadedWorld, "Patrol");
    ASSERT_TRUE(reloadedPatrol.has_value());
    const WaypointComponent& reloaded = reloadedWorld.Components<WaypointComponent>().Get(*reloadedPatrol);
    ASSERT_EQ(reloaded.queue.size(), shown.size());
    for (size_t i = 0; i < shown.size(); ++i) {
        EXPECT_TRUE(NearlyEqual(reloaded.queue[i].pos.x(), shown[i].pos.x())) << "point " << i;
        EXPECT_TRUE(NearlyEqual(reloaded.queue[i].pos.z(), shown[i].pos.z())) << "point " << i;
    }
}

}  // namespace

TEST(ScenarioSaver, ARotatedLoopWithAPointAddedSavesAsShown) {
    ExpectEditedRotatedLoopSavesAsShown([](WaypointComponent& waypoint) {
        WaypointSystem::AppendPoint(waypoint, WaypointPoint{Vec3{1234.0, 25000.0, 5678.0}, std::nullopt});
    });
}

TEST(ScenarioSaver, ARotatedLoopTruncatedSavesAsShown) {
    ExpectEditedRotatedLoopSavesAsShown(
        [](WaypointComponent& waypoint) { WaypointSystem::TruncateQueue(waypoint, waypoint.queue.size() - 1); });
}

// --- A partial loop flies its lead-in once, then repeats only the points
// from loop_from on: A B C D, loop_from 1 -> A B C D B C D B ...
TEST(ScenarioSaver, PartialLoopRepeatsOnlyFromLoopFrom) {
    World world;
    ScenarioData scenario;
    ASSERT_NO_THROW({ scenario = ScenarioLoader::Load(kSourcePath, world); });
    std::optional<EntityID> patrol = FindByTagName(world, "Patrol");
    ASSERT_TRUE(patrol.has_value());
    WaypointComponent& waypoint = world.Components<WaypointComponent>().Get(*patrol);
    waypoint.loop = true;
    waypoint.loopFrom = 1;
    std::vector<WaypointPoint> authored(waypoint.queue.begin(), waypoint.queue.end());
    const size_t n = authored.size();
    ASSERT_GE(n, 3u);

    // Arrival order over three laps of the repeating part.
    std::vector<size_t> expected{0};
    for (int lap = 0; lap < 3; ++lap) {
        for (size_t i = 1; i < n; ++i) expected.push_back(i);
    }
    for (size_t step = 0; step < expected.size(); ++step) {
        const Vec3& want = authored[expected[step]].pos;
        const Vec3& front = waypoint.queue.front().pos;
        EXPECT_TRUE(NearlyEqual(front.x(), want.x()) && NearlyEqual(front.z(), want.z()))
            << "arrival " << step << " should be authored point " << expected[step];
        world.Components<Transform>().Get(*patrol).pos = front;
        WaypointSystem::Update(world, scenario.environment);
    }
    EXPECT_EQ(waypoint.queue.size(), n - 1);
}

// --- Save must fail cleanly (not throw/crash) when the source scenario
// path doesn't exist.
TEST(ScenarioSaver, MissingSourceFailsCleanly) {
    World world;
    ScenarioData scenario;
    std::string error;
    bool saved = ScenarioSaver::Save("scenarios/compound/scenario_this_does_not_exist.json", ScratchPath(), world,
                                      scenario, error);
    std::filesystem::remove(ScratchPath());
    EXPECT_FALSE(saved);
    EXPECT_FALSE(error.empty());
}

// A flag or enum the file sets is compared against the file's value, not
// the type's: setting it back to the type's value must still be saved, or
// the file's old value would come back on reload.
TEST(ScenarioSaver, FileValueIsTheBaselineForFlagsAndEnums) {
    const ParamRegistry::ParamDef* mode = ParamRegistry::Find("/missile_doctrine/launch_mode");
    const ParamRegistry::ParamDef* loft = ParamRegistry::Find("/missile_doctrine/loft");
    ASSERT_NE(mode, nullptr);
    ASSERT_NE(loft, nullptr);

    nlohmann::json source = ScenarioLoader::ReadJsonFile("tests/fixtures/scenarios/scenario_basic.json");
    World typeWorld;
    ScenarioData typeDefaults = ScenarioLoader::LoadFromJson(source, typeWorld);
    const int typeMode = mode->enumAccess.get(typeDefaults, typeWorld, kNoEntity);
    const bool typeLoft = *loft->flag(typeDefaults, typeWorld, kNoEntity);

    // The file overrides both away from the type's values.
    source["missile_doctrine"]["launch_mode"] = mode->enumAccess.names[typeMode == 0 ? 1 : 0];
    source["missile_doctrine"]["loft"] = !typeLoft;
    const std::string sourcePath = ScratchPath() + ".source.json";
    std::ofstream(sourcePath) << source.dump(2);

    World world;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(source, world);
    nlohmann::json unchanged;
    std::string error;
    ASSERT_TRUE(ScenarioSaver::BuildJson(sourcePath, world, scenario, unchanged, error)) << error;
    EXPECT_EQ(unchanged["missile_doctrine"]["launch_mode"], source["missile_doctrine"]["launch_mode"]);
    EXPECT_EQ(unchanged["missile_doctrine"]["loft"], !typeLoft);

    // Back to the type's values: both must be written.
    mode->enumAccess.set(scenario, world, kNoEntity, typeMode);
    *loft->flag(scenario, world, kNoEntity) = typeLoft;
    nlohmann::json edited;
    ASSERT_TRUE(ScenarioSaver::BuildJson(sourcePath, world, scenario, edited, error)) << error;
    std::filesystem::remove(sourcePath);
    EXPECT_EQ(edited["missile_doctrine"]["launch_mode"], mode->enumAccess.names[typeMode]);
    EXPECT_EQ(edited["missile_doctrine"]["loft"], typeLoft);
}

TEST(ScenarioSaver, SourceWithoutEntitiesFailsCleanly) {
    const std::string sourcePath = ScratchPath() + ".source.json";
    std::ofstream(sourcePath) << R"({"seed": 1, "sim_speed": 1.0})";
    World world;
    ScenarioData scenario;
    nlohmann::json out;
    std::string error;
    bool built = ScenarioSaver::BuildJson(sourcePath, world, scenario, out, error);
    std::filesystem::remove(sourcePath);
    EXPECT_FALSE(built);
    EXPECT_NE(error.find("no \"entities\" array"), std::string::npos) << error;
}

// JSON has no NaN or infinity: dumped, they become null, which the loader
// rejects, so a file saved from a bad live value would never load again.
TEST(ScenarioSaver, ANonFiniteLiveValueRefusesToSave) {
    // The basic scenario has a missile_doctrine block; the patrol fixture has none.
    const char* basicPath = "tests/fixtures/scenarios/scenario_basic.json";
    World world;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(ScenarioLoader::ReadJsonFile(basicPath), world);
    const ParamRegistry::ParamDef* gain = ParamRegistry::Find("/missile_doctrine/nav_gain");
    ASSERT_NE(gain, nullptr);
    *gain->dbl(scenario, world, kNoEntity) = std::numeric_limits<double>::quiet_NaN();
    nlohmann::json out;
    std::string error;
    EXPECT_FALSE(ScenarioSaver::BuildJson(basicPath, world, scenario, out, error));
    EXPECT_NE(error.find("nav_gain"), std::string::npos) << error;
    EXPECT_NE(error.find("not a finite number"), std::string::npos) << error;
}

TEST(ScenarioSaver, ANonFiniteWaypointRefusesToSave) {
    World world;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(ScenarioLoader::ReadJsonFile(kSourcePath), world);
    std::optional<EntityID> patrol = FindByTagName(world, "Patrol");
    ASSERT_TRUE(patrol.has_value());
    world.Components<WaypointComponent>().Get(*patrol).queue.front().pos =
        Vec3(std::numeric_limits<double>::infinity(), 0.0, 0.0);
    nlohmann::json out;
    std::string error;
    EXPECT_FALSE(ScenarioSaver::BuildJson(kSourcePath, world, scenario, out, error));
    EXPECT_NE(error.find("waypoint"), std::string::npos) << error;
}

TEST(ScenarioSaver, UnwritableDestinationFailsCleanly) {
    World world;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(ScenarioLoader::ReadJsonFile(kSourcePath), world);
    std::string error;
    EXPECT_FALSE(ScenarioSaver::Save(kSourcePath, ::testing::TempDir() + "no_such_dir/out.json", world, scenario,
                                     error));
    EXPECT_NE(error.find("Could not open destination path"), std::string::npos) << error;
}

// Reset rebuilds into a scratch world, so a file that no longer loads
// leaves the running world as it was.
TEST(ScenarioSaver, FailedResetLeavesTheWorldUntouched) {
    World world;
    ScenarioData scenario = ScenarioLoader::LoadFromJson(ScenarioLoader::ReadJsonFile(kSourcePath), world);
    const size_t entities = world.Entities().size();
    ASSERT_GT(entities, 0u);

    nlohmann::json broken = ScenarioLoader::ReadJsonFile(kSourcePath);
    broken["made_up_block"] = nlohmann::json::array();
    const std::string sourcePath = ScratchPath() + ".source.json";
    std::ofstream(sourcePath) << broken.dump(2);
    std::string error;
    bool reset = ScenarioSaver::ResetWorld(sourcePath, world, scenario, error);
    std::filesystem::remove(sourcePath);
    EXPECT_FALSE(reset);
    EXPECT_NE(error.find("Failed to reload edited scenario"), std::string::npos) << error;
    EXPECT_EQ(world.Entities().size(), entities);
    EXPECT_TRUE(FindByTagName(world, "Patrol").has_value());
}

}  // namespace

// Save As names stay inside scenarios/custom/ and never shadow the template.
TEST(ScenarioSaver, CustomFileNameRefusesPathsAndReservedNames) {
    std::string error;
    EXPECT_EQ(ScenarioSaver::CustomFileName("my_run-2", error), "my_run-2.json");
    EXPECT_EQ(ScenarioSaver::CustomFileName("my_run.json", error), "my_run.json");
    for (const char* bad : {"", ".json", "../1_core_01_first_shot", "sub/name", "a\b", "C:x", "two words",
                            "_template", "_template.json", "name.txt"}) {
        EXPECT_EQ(ScenarioSaver::CustomFileName(bad, error), "") << bad;
        EXPECT_FALSE(error.empty()) << bad;
    }
}
