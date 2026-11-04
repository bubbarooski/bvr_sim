// A missile type's launch zone: its maximum launch range, Rmax, over a grid
// of launch conditions -- the farthest range at which the type's own flight
// model still reaches a target that does not maneuver (Dantas et al.,
// arXiv:2111.04474, Section 2.1).
//
// Each launch is the sim's own: LaunchSystem::Init and SimLoop::Step at the
// sim's fixed step, so the missile flies exactly the motor, aerodynamics,
// guidance, seeker, self-destruct and fuze a scenario would. What is taken
// away is everything that is not the missile's kinematic reach:
//   - the launcher's radar track is the target's true state (no measurement
//     error, never lost), relayed over the datalink as usual;
//   - the type's guidance response time, which is part of the missile;
//   - a standard day, no wind;
//   - the seeker's Doppler notch (a beam-aspect cell would otherwise be
//     notched: a sensor limit, not reach);
//   - the target an F-16 (its hit box and radar signature) flying straight
//     and level at constant speed, at the shooter's altitude;
//   - the shooter flying straight and level, the target off its nose at the
//     cell's off-boresight angle.
// The seeker's detections are still random (Swerling 1), so every launch
// uses the same seed and the table is deterministic.
//
// "Reaches" = the engagement resolves with the closest approach to the
// target's airframe within the kill radius. Doctrine values are the ones
// every scenario in scenarios/ uses: nav gain 5, kill radius 12 m, max
// range 150 km.
//
// Range search per cell: steps of 5 nmi out from 5 nmi to the first miss,
// then bisection to 0.1 nmi. A zone that is not one interval (a miss
// inside Rmax) is not looked for.
//
// Output: data/missiles/<id>/fit/launch_zone.json, loaded by
// MissileCatalog. Run from the repo root:
//   build/Release/tools/launch_zone.exe --missile aim120 [--threads N]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "aircraft/AircraftCatalog.h"
#include "aircraft/AircraftEntity.h"
#include "components/AircraftComponent.h"
#include "components/AirborneRadarTrack.h"
#include "components/KinematicsComponent.h"
#include "components/RadarCrossSection.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"
#include "missile/LaunchZone.h"
#include "missile/MissileCatalog.h"
#include "scenario/ScenarioLoader.h"
#include "sim/Atmosphere.h"
#include "sim/SimClock.h"
#include "sim/SimContext.h"
#include "sim/SimLoop.h"
#include "systems/LaunchSystem.h"
#include "util/Units.h"
#include "util/Vec3.h"

namespace {

constexpr const char* kTargetAircraft = "f16c_block50";
constexpr double kNavGain = 5.0;
constexpr double kKillRadiusM = 12.0;
constexpr double kMaxRangeM = 150000.0;
constexpr uint64_t kSeed = 1;
constexpr double kStepNmi = 5.0;
constexpr double kResolutionNmi = 0.1;
// Beyond the doctrine's max range guidance gives up at launch anyway.
constexpr double kSearchCeilingNmi = kMaxRangeM / Units::kMetersPerNmi;
// A launch is flown this long past the self-destruct time of flight.
constexpr double kTimeoutMarginS = 10.0;
constexpr double kRmaxPerHundredthNmi = 100.0;  // rmax is written to 0.01 nmi
constexpr int kProgressEveryCells = 50;
constexpr int kJsonIndent = 1;
constexpr int kProbeArgs = 6;  // values after --probe

const std::vector<double> kShooterAltitudeFt = {5000.0, 15000.0, 25000.0, 35000.0, 45000.0};
const std::vector<double> kShooterMach = {0.6, 0.8, 1.0, 1.2, 1.4};
const std::vector<double> kTargetMach = {0.6, 0.8, 1.0, 1.2, 1.4};
const std::vector<double> kAspectDeg = {-180.0, -150.0, -120.0, -90.0, -60.0, -30.0, 0.0,
                                        30.0,   60.0,   90.0,   120.0,  150.0,  180.0};
// Out to the radar's +/-60 deg gimbal. A target outside the missile's
// seeker cone at launch is lost (see GuidanceSystem), so Rmax has a cliff
// there: grid points either side of the type's cone keep the interpolation
// from smearing it.
constexpr double kRadarGimbalDeg = 60.0;
constexpr double kConeEdgeOffsetDeg = 1.0;
std::vector<double> OffBoresightAxisDeg(const MissileType& type) {
    std::vector<double> axis = {0.0, 15.0, 30.0, kRadarGimbalDeg};
    for (double edge : {type.seeker.fovHalfAngleDeg - kConeEdgeOffsetDeg,
                        type.seeker.fovHalfAngleDeg + kConeEdgeOffsetDeg}) {
        if (edge > 0.0 && edge < kRadarGimbalDeg && std::find(axis.begin(), axis.end(), edge) == axis.end()) axis.push_back(edge);
    }
    std::sort(axis.begin(), axis.end());
    return axis;
}

struct Cell {
    double altitudeFt, shooterMach, targetMach, aspectDeg, offBoresightDeg;
};

ScenarioData ReferenceDoctrine(const std::shared_ptr<const MissileType>& type) {
    ScenarioData scenario;
    auto noNotch = std::make_shared<MissileType>(*type);
    noNotch->seeker.notchVelocityThresholdMps = 0.0;
    scenario.SetMissileType(noNotch);
    scenario.missileNavGain = kNavGain;
    scenario.killRadiusFt = kKillRadiusM * Units::kFeetPerMeter;
    scenario.maxRangeFt = kMaxRangeM * Units::kFeetPerMeter;
    return scenario;
}

void SetTrackToTruth(World& world, AirborneRadarTrack& track, double simTime) {
    const Vec3& pos = world.Components<Transform>().Get(track.target).pos;
    const Vec3& vel = world.Components<KinematicsComponent>().Get(track.target).vel;
    track.estimate.valid = true;
    track.estimate.time = simTime;
    track.estimate.state = {pos.x(), pos.y(), pos.z(), vel.x(), vel.y(), vel.z()};
}

EntityID SpawnStraightFlyer(World& world, const char* tag, const Vec3& pos, const Vec3& vel) {
    EntityID entity = world.CreateEntity();
    world.Components<Tag>().Add(entity, Tag{tag});
    double yawDeg = std::atan2(vel.x(), vel.z()) * Units::kRadToDeg;
    world.Components<Transform>().Add(entity, Transform{pos, Vec3{0.0, 0.0, yawDeg}});
    KinematicsComponent kinematics;
    kinematics.vel = vel;
    kinematics.speed = vel.norm();
    world.Components<KinematicsComponent>().Add(entity, kinematics);
    return entity;
}

// One launch at `rangeNmi`: true if the missile reaches the target. With
// `report`, prints how the engagement ended.
bool Reaches(const ScenarioData& doctrine, const std::shared_ptr<const AircraftType>& targetType, const Cell& cell,
             double rangeNmi, bool report = false) {
    ScenarioData scenario = doctrine;
    World world;
    SimContext simContext;
    SimLoop::BeginRun(scenario, simContext, kSeed);

    double speedOfSound = Atmosphere::Lookup(cell.altitudeFt).speedOfSoundFtPerSec;
    // Shooter at the origin heading +Z; the target off its nose at the
    // off-boresight angle, flying so that the shooter sits at `aspect` from
    // its tail (LaunchZone::Geometry's convention, inverted).
    Vec3 shooterVel{0.0, 0.0, cell.shooterMach * speedOfSound};
    EntityID shooter = SpawnStraightFlyer(world, "Pursuer", Vec3{0.0, cell.altitudeFt, 0.0}, shooterVel);
    double bearingRad = cell.offBoresightDeg * Units::kDegToRad;
    double rangeFt = rangeNmi * Units::kFeetPerNmi;
    Vec3 targetPos{rangeFt * std::sin(bearingRad), cell.altitudeFt, rangeFt * std::cos(bearingRad)};
    double targetHeadingRad = (cell.offBoresightDeg - cell.aspectDeg) * Units::kDegToRad;
    double targetSpeed = cell.targetMach * speedOfSound;
    Vec3 targetVel{targetSpeed * std::sin(targetHeadingRad), 0.0, targetSpeed * std::cos(targetHeadingRad)};
    EntityID target = SpawnStraightFlyer(world, "Target", targetPos, targetVel);
    world.Components<AircraftComponent>().Add(target, AircraftComponent{targetType});
    world.Components<RadarCrossSection>().Add(target, AircraftEntity::MakeRadarCrossSection(*targetType));

    LaunchSystem::Init(world, shooter, target, scenario);

    // A perfect, never-lost launcher track, and no range gate -- this tool
    // is what finds the range the gate uses: LaunchSystem fires on the first
    // tick (the shooter carries no radar, so no fire-control delay either),
    // and the datalink relays the truth.
    scenario.launchRangeFraction = std::numeric_limits<double>::infinity();
    AirborneRadarTrack& track = simContext.airborneRadarTracks.FindOrCreate(shooter, target);
    track.owner = shooter;
    track.target = target;
    track.state = RadarTrackState::CONFIRMED;
    double simTime = 0.0;
    SetTrackToTruth(world, track, simTime);

    const double limitS = scenario.missileSelfDestructMaxTimeOfFlightS + kTimeoutMarginS;
    while (!simContext.outcome.resolved && simTime < limitS) {
        SimLoop::Step(world, simContext, scenario, SimClock::kFixedDt, simTime);
        SetTrackToTruth(world, simContext.airborneRadarTracks.FindOrCreate(shooter, target), simTime);
    }
    const EngagementOutcome& outcome = simContext.outcome;
    if (!outcome.launched) throw std::logic_error("launch_zone: the missile never launched");
    if (report) {
        std::printf("range %.2f nmi: resolved %d, miss %.1f m, region %s, time of flight %.1f s, seeker active %.1f s\n",
                    rangeNmi, outcome.resolved ? 1 : 0, outcome.missDistanceFt / Units::kFeetPerMeter,
                    outcome.impactRegion.c_str(), outcome.resolutionSimTime - outcome.launchSimTime,
                    outcome.seekerActiveSimTime - outcome.launchSimTime);
    }
    return outcome.resolved && outcome.missDistanceFt <= scenario.killRadiusFt;
}

double RmaxNmi(const ScenarioData& doctrine, const std::shared_ptr<const AircraftType>& targetType, const Cell& cell) {
    double lo = 0.0, hi = 0.0;
    for (double r = kStepNmi;; r += kStepNmi) {
        if (r > kSearchCeilingNmi) return kSearchCeilingNmi;
        if (!Reaches(doctrine, targetType, cell, r)) {
            hi = r;
            break;
        }
        lo = r;
    }
    while (hi - lo > kResolutionNmi) {
        double mid = 0.5 * (lo + hi);
        (Reaches(doctrine, targetType, cell, mid) ? lo : hi) = mid;
    }
    return lo;
}

int ParseThreads(int argc, char** argv) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--threads") == 0) return std::max(1, std::atoi(argv[i + 1]));
    }
    return std::max(1u, std::thread::hardware_concurrency());
}

std::string ParseMissile(int argc, char** argv) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], "--missile") == 0) return argv[i + 1];
    }
    throw std::runtime_error("usage: launch_zone --missile <id> [--threads N]");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::string missileId = ParseMissile(argc, argv);
        const int threads = ParseThreads(argc, argv);
        std::shared_ptr<const MissileType> type = MissileCatalog::Default().Get(missileId);
        std::shared_ptr<const AircraftType> targetType = AircraftCatalog::Default().Get(kTargetAircraft);
        const ScenarioData doctrine = ReferenceDoctrine(type);

        // --probe alt_ft shooter_mach target_mach aspect_deg off_boresight_deg
        // range_nmi: one launch, reported; nothing written.
        for (int i = 1; i + kProbeArgs < argc; ++i) {
            if (std::strcmp(argv[i], "--probe") != 0) continue;
            Cell cell{std::atof(argv[i + 1]), std::atof(argv[i + 2]), std::atof(argv[i + 3]), std::atof(argv[i + 4]),
                      std::atof(argv[i + 5])};
            auto started = std::chrono::steady_clock::now();
            bool reached = Reaches(doctrine, targetType, cell, std::atof(argv[i + 6]), true);
            std::printf("reaches %d (%.2f s wall)\n", reached ? 1 : 0,
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
            return 0;
        }

        const std::vector<double> offBoresightDeg = OffBoresightAxisDeg(*type);
        const std::array<const std::vector<double>*, LaunchZone::kAxes> axes = {
            &kShooterAltitudeFt, &kShooterMach, &kTargetMach, &kAspectDeg, &offBoresightDeg};
        std::vector<Cell> cells;
        for (double alt : kShooterAltitudeFt)
            for (double ms : kShooterMach)
                for (double mt : kTargetMach)
                    for (double aspect : kAspectDeg)
                        for (double off : offBoresightDeg) cells.push_back(Cell{alt, ms, mt, aspect, off});

        // Cells another cell already answers: aspect -180 is +180, and with
        // the target dead ahead a negative aspect is the mirror of the
        // positive one. -1 = fly it.
        std::vector<long> sameAs(cells.size(), -1);
        auto indexOf = [&](const Cell& c) {
            for (size_t i = 0; i < cells.size(); ++i) {
                const Cell& o = cells[i];
                if (o.altitudeFt == c.altitudeFt && o.shooterMach == c.shooterMach && o.targetMach == c.targetMach &&
                    o.aspectDeg == c.aspectDeg && o.offBoresightDeg == c.offBoresightDeg)
                    return static_cast<long>(i);
            }
            throw std::logic_error("launch_zone: cell not on the grid");
        };
        for (size_t i = 0; i < cells.size(); ++i) {
            Cell c = cells[i];
            if (c.aspectDeg == -180.0) {
                c.aspectDeg = 180.0;
                sameAs[i] = indexOf(c);
            } else if (c.offBoresightDeg == 0.0 && c.aspectDeg < 0.0) {
                c.aspectDeg = -c.aspectDeg;
                sameAs[i] = indexOf(c);
            }
        }

        std::vector<double> rmax(cells.size(), -1.0);
        std::atomic<size_t> next{0}, done{0};
        size_t toFly = static_cast<size_t>(std::count(sameAs.begin(), sameAs.end(), -1));
        auto started = std::chrono::steady_clock::now();
        std::vector<std::thread> pool;
        for (int t = 0; t < threads; ++t) {
            pool.emplace_back([&]() {
                for (size_t i = next++; i < cells.size(); i = next++) {
                    if (sameAs[i] >= 0) continue;
                    rmax[i] = RmaxNmi(doctrine, targetType, cells[i]);
                    size_t n = ++done;
                    if (n % kProgressEveryCells == 0 || n == toFly) {
                        double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
                        std::fprintf(stderr, "  %zu / %zu cells, %.0f s\n", n, toFly, s);
                    }
                }
            });
        }
        for (std::thread& th : pool) th.join();
        for (size_t i = 0; i < cells.size(); ++i) {
            if (sameAs[i] >= 0) rmax[i] = rmax[static_cast<size_t>(sameAs[i])];
        }

        const std::filesystem::path missileFile = type->directory / "missile.json";
        nlohmann::ordered_json out;
        out["generated_by"] = "tools/drivers/launch_zone.cpp";
        out["missile"] = missileId;
        out["missile_file_hash"] = LaunchZone::FileHash(missileFile);
        out["definition"] =
            "Rmax: the farthest launch range at which the missile reaches a non-maneuvering target "
            "(Dantas et al., arXiv:2111.04474). Reaches = resolves with closest approach to the airframe "
            "within the kill radius, before self-destruct.";
        out["conditions"] = {
            {"target_aircraft", kTargetAircraft},
            {"target_flight", "straight and level at constant speed, at the shooter's altitude"},
            {"shooter_flight", "straight and level"},
            {"launcher_track", "truth, never lost"},
            {"response_time", "the type's"},
            {"atmosphere", "standard day, no wind"},
            {"nav_gain", kNavGain},
            {"kill_radius_m", kKillRadiusM},
            {"max_range_m", kMaxRangeM},
            {"seed", kSeed},
            {"time_step_s", SimClock::kFixedDt},
            {"range_resolution_nmi", kResolutionNmi}};
        out["axes"] = nlohmann::ordered_json::object();
        for (int a = 0; a < LaunchZone::kAxes; ++a) out["axes"][LaunchZone::kAxisNames[a]] = *axes[a];
        out["rmax_nmi"] = nlohmann::ordered_json::array();
        for (double r : rmax) out["rmax_nmi"].push_back(std::round(r * kRmaxPerHundredthNmi) / kRmaxPerHundredthNmi);

        const std::filesystem::path outFile = type->directory / "fit" / "launch_zone.json";
        std::filesystem::create_directories(outFile.parent_path());
        std::ofstream file(outFile, std::ios::binary);
        file << out.dump(kJsonIndent) << '\n';
        std::fprintf(stderr, "wrote %s\n", outFile.string().c_str());
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "launch_zone: %s\n", e.what());
        return 1;
    }
}
