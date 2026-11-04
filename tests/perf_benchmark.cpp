// Headless performance benchmark for the sim-side systems -- the part of
// each tick whose cost scales with aircraft count. Deliberately uses
// KINEMATIC aircraft (Velocity, not JSBSim): JSBSim's per-aircraft Run()
// cost is O(N) and lives in a vendored dependency this can't touch, so
// isolating it out makes the systems actually under optimization visible.
//
// Every aircraft carries an airborne radar AND a radar cross-section, so
// AirborneRadarSystem evaluates all N*(N-1) ordered pairs every sweep --
// the O(N^2) term that decides whether "more aircraft" stays smooth.
//
// Prints wall time and ticks/s, plus a behavior checksum (event count +
// summed final positions) so a run before and after an optimization can be
// confirmed to have computed the *same thing*, not just faster -- the
// validation suite's scenarios never exercise this many entities.
//
// Usage: perf_benchmark [entities] [ticks]   (defaults: 40 3000)

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "aircraft/AircraftCatalog.h"
#include "aircraft/AircraftEntity.h"
#include "components/AirborneRadarComponent.h"
#include "components/RadarCrossSection.h"
#include "components/Transform.h"
#include "components/Velocity.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "scenario/ScenarioLoader.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"
#include "sim/SimLoop.h"
#include "util/Units.h"

namespace {

// The F-16's radar (its cone, gates and track logic) with a receiver budget
// sized so a good fraction of the in-FOV pairs actually detect at the grid
// spacing below -- that keeps the track state machine and its event
// bookkeeping on the measured path, not just the equation.
void AddAirborneRadar(World& world, EntityID e) {
    AirborneRadarComponent radar = AircraftEntity::MakeAirborneRadar(*AircraftCatalog::Default().Get("f16c_block50"));
    radar.frameTimeS = SimClock::kFixedDt;  // a frame every tick: maximal O(N^2) stress
    radar.averagePowerW = 15000.0;
    radar.peakPowerW = 15000.0;
    radar.antennaGain = 4000.0;
    radar.frequencyHz = Units::kSpeedOfLightMPerS / 0.03;
    radar.timeOnTargetS = 1.0;  // with no noise figure or loss, SNR = received power / kT0
    radar.noiseFigureDb = 0.0;
    radar.systemLossDb = 0.0;
    // The benchmark's original received-power thresholds, as SNR.
    auto snrDbForPowerW = [](double powerW) {
        return 10.0 * std::log10(powerW / (Units::kBoltzmannJPerK * Units::kReferenceNoiseTempK));
    };
    radar.searchThresholdDb = snrDbForPowerW(1.0e-13);
    world.Components<AirborneRadarComponent>().Add(e, radar);
}

}  // namespace

int main(int argc, char** argv) {
    int entityCount = argc > 1 ? std::atoi(argv[1]) : 40;
    int ticks = argc > 2 ? std::atoi(argv[2]) : 3000;
    if (entityCount < 2) entityCount = 2;
    if (ticks < 1) ticks = 1;

    World world;
    SimContext simContext;
    simContext.rng.Seed(1234567);

    // Lay the aircraft out on a rough square grid, a few km apart, drifting
    // slowly. Spacing is chosen so neighbours sit inside detection range
    // and the diagonal does not, giving a realistic mix of hits/misses.
    int side = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(entityCount))));
    const double spacingFt = 20000.0;
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    for (int i = 0; i < entityCount; ++i) {
        EntityID e = world.CreateEntity();
        int row = i / side, col = i % side;

        Transform t;
        t.pos = Vec3{col * spacingFt, 20000.0 + (i % 5) * 500.0, row * spacingFt};
        // Headings fan out so view directions (and thus RCS) vary per pair.
        t.euler = Vec3{0.0, 0.0, static_cast<double>((i * 47) % 360)};
        transforms.Add(e, t);

        Velocity v;
        double headingRad = t.euler.z() * 3.14159265358979323846 / 180.0;
        v.linear = Vec3{std::sin(headingRad) * 800.0, 0.0, std::cos(headingRad) * 800.0};
        world.Components<Velocity>().Add(e, v);

        // The F-16 type's shape, so every pair pays the projected-area cost.
        RadarCrossSection rcs = AircraftEntity::MakeRadarCrossSection(*AircraftCatalog::Default().Get("f16c_block50"));
        world.Components<RadarCrossSection>().Add(e, rcs);

        AddAirborneRadar(world, e);
    }

    ScenarioData scenario;  // defaults are enough: no missile engagement here

    double simTime = 0.0;
    auto start = std::chrono::steady_clock::now();
    for (int tick = 0; tick < ticks; ++tick) {
        SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
    }
    auto end = std::chrono::steady_clock::now();

    double wallMs = std::chrono::duration<double, std::milli>(end - start).count();
    double ticksPerSec = ticks / (wallMs / 1000.0);

    // Behavior checksum: number of logged events plus the summed final
    // positions of every aircraft. Any change in what the sim computes
    // moves one of these; identical values before/after mean the
    // optimization was behavior-preserving on this many-entity path.
    double posChecksum = 0.0;
    for (auto& [entity, t] : transforms) posChecksum += t.pos.x() + t.pos.y() + t.pos.z();
    std::size_t eventCount = simContext.eventLog.Events().size();

    std::printf("entities=%d ticks=%d wall_ms=%.1f ticks_per_sec=%.1f events=%zu pos_checksum=%.6f\n",
                entityCount, ticks, wallMs, ticksPerSec, eventCount, posChecksum);
    return 0;
}
