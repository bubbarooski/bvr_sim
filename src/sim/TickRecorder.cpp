#include "sim/TickRecorder.h"

#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>

#include "FGFDMExec.h"
#include "models/FGAuxiliary.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGEngine.h"

#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "components/AutopilotComponent.h"
#include "components/DatalinkTrackComponent.h"
#include "components/DestroyedComponent.h"
#include "components/GLimitComponent.h"
#include "components/GuidanceComponent.h"
#include "components/JSBSimComponent.h"
#include "components/KinematicsComponent.h"
#include "components/RwrComponent.h"
#include "components/Tag.h"
#include "components/Transform.h"
#include "components/Velocity.h"
#include "components/WaypointComponent.h"
#include "ecs/ComponentStore.h"
#include "sim/Atmosphere.h"
#include "sim/Environment.h"
#include "sim/HitWindow.h"
#include "sim/RadarTrackFilter.h"
#include "sim/SimContext.h"
#include "systems/GuidanceSystem.h"
#include "util/EntityVelocity.h"
#include "util/Units.h"

using Units::kGravityFtPerSec2;
using Units::kRadToDeg;

namespace {

// The one list of columns: X(enum name, header text). Order is file order.
// Frames and units: docs/SCHEMA.md, "Tick recording".
#define TICK_RECORDER_COLUMNS(X)                                                                          \
    X(SimTime, "sim_time_s")                                                                              \
    X(Kind, "kind")                                                                                       \
    X(Entity, "entity")                                                                                   \
    X(Name, "name")                                                                                       \
    X(Target, "target")                                                                                   \
    X(Destroyed, "destroyed")                                                                             \
    X(PosX, "x_ft") X(PosY, "y_ft") X(PosZ, "z_ft")                                                       \
    X(Roll, "roll_deg") X(Pitch, "pitch_deg") X(Yaw, "yaw_deg")                                           \
    X(VelX, "vx_fps") X(VelY, "vy_fps") X(VelZ, "vz_fps")                                                 \
    X(Speed, "speed_fps")                                                                                 \
    X(Mach, "mach")                                                                                       \
    X(Kcas, "kcas")                                                                                       \
    X(Alpha, "alpha_deg") X(Beta, "beta_deg")                                                             \
    X(Qbar, "qbar_psf")                                                                                   \
    X(Nz, "nz_g") X(Nx, "nx_g") X(Ny, "ny_g")                                                             \
    X(RollRate, "p_deg_s") X(PitchRate, "q_deg_s")                                                        \
    X(Weight, "weight_lbf") X(Fuel, "fuel_lbf")                                                           \
    X(Thrust, "thrust_lbf")                                                                               \
    X(AileronCmd, "aileron_cmd") X(ElevatorCmd, "elevator_cmd") X(ThrottleCmd, "throttle_cmd")            \
    X(ApHeading, "ap_heading_deg") X(ApAltitude, "ap_altitude_ft") X(ApSpeed, "ap_speed_kts")             \
    X(Damage, "overstress_damage")                                                                        \
    X(WaypointsLeft, "waypoints_left")                                                                    \
    X(SttTarget, "radar_stt_target")                                                                      \
    X(RwrMode, "rwr_mode") X(RwrIlluminated, "rwr_illuminated") X(RwrMissileWarning, "rwr_missile_warning") \
    X(AccelX, "ax_fps2") X(AccelY, "ay_fps2") X(AccelZ, "az_fps2")                                        \
    X(AchievedG, "achieved_lateral_g")                                                                    \
    X(MassSlug, "mass_slug")                                                                              \
    X(Launched, "launched") X(Active, "guidance_active")                                                  \
    X(SeekerMode, "seeker_mode") X(SeekerActive, "seeker_active")                                         \
    X(HasLock, "has_lock") X(TimeSinceLockLost, "time_since_lock_lost_s")                                 \
    X(LaunchTurnDone, "launch_turn_done") X(Lofting, "lofting")                                           \
    X(RangeToTarget, "range_to_target_ft")                                                                \
    X(TrackState, "track_state") X(FrameHits, "frame_hits") X(MissedFrames, "missed_frames")              \
    X(EstAge, "est_age_s") X(LinkLost, "datalink_lost")                                                   \
    X(EstX, "est_x_ft") X(EstY, "est_y_ft") X(EstZ, "est_z_ft")                                           \
    X(EstVelX, "est_vx_fps") X(EstVelY, "est_vy_fps") X(EstVelZ, "est_vz_fps")                            \
    X(EstPosSigma, "est_pos_sigma_ft")                                                                    \
    X(EstError, "est_error_ft")

enum class Col : std::size_t {
#define X(id, header) id,
    TICK_RECORDER_COLUMNS(X)
#undef X
    Count
};

constexpr const char* kHeaders[] = {
#define X(id, header) header,
    TICK_RECORDER_COLUMNS(X)
#undef X
};

// One row's cells as text; an unset cell stays empty.
class Row {
public:
    void Set(Col col, double value) {
        if (!std::isfinite(value)) return;
        // Nine significant digits: 0.01 ft at a million feet, and shortest
        // round-trip text below that, so a small value is not padded.
        char buffer[32];
        std::to_chars_result result =
            std::to_chars(buffer, buffer + sizeof(buffer), value, std::chars_format::general, 9);
        cells_[Index(col)].assign(buffer, result.ptr);
    }
    void Set(Col col, int value) { cells_[Index(col)] = std::to_string(value); }
    void Set(Col col, unsigned int value) { cells_[Index(col)] = std::to_string(value); }
    void Set(Col col, bool value) { cells_[Index(col)] = value ? "1" : "0"; }
    void Set(Col col, const char* value) { cells_[Index(col)] = value; }
    void SetName(const std::string& name) {
        std::string& cell = cells_[Index(Col::Name)];
        if (name.find_first_of(",\"\n\r") == std::string::npos) {
            cell = name;
            return;
        }
        cell = "\"";
        for (char c : name) {
            if (c == '"') cell += '"';
            cell += c;
        }
        cell += '"';
    }
    void SetVec(Col first, const Vec3& v) {
        Set(first, v.x());
        Set(static_cast<Col>(Index(first) + 1), v.y());
        Set(static_cast<Col>(Index(first) + 2), v.z());
    }

    void AppendTo(std::string& line) {
        for (std::size_t i = 0; i < cells_.size(); ++i) {
            if (i > 0) line += ',';
            line += cells_[i];
            cells_[i].clear();
        }
        line += '\n';
    }

private:
    static std::size_t Index(Col col) { return static_cast<std::size_t>(col); }
    std::array<std::string, static_cast<std::size_t>(Col::Count)> cells_;
};

const char* RwrModeName(RwrMode mode) {
    switch (mode) {
        case RwrMode::SEARCH: return "SEARCH";
        case RwrMode::LOCK: return "LOCK";
        case RwrMode::NONE: break;
    }
    return "NONE";
}

const char* SeekerModeName(SeekerMode mode) {
    switch (mode) {
        case SeekerMode::SARH: return "SARH";
        case SeekerMode::MIDCOURSE_ARH: return "MIDCOURSE_ARH";
        case SeekerMode::ARH: break;
    }
    return "ARH";
}

const char* TrackStateName(RadarTrackState state) {
    switch (state) {
        case RadarTrackState::TENTATIVE: return "TENTATIVE";
        case RadarTrackState::CONFIRMED: return "CONFIRMED";
        case RadarTrackState::NONE: break;
    }
    return "NONE";
}

void AddAircraftColumns(Row& row, const JSBSimComponent& jsbsim) {
    const JSBSimPropertyNodes& props = jsbsim.props;
    const JSBSim::FGFDMExec& fdm = *jsbsim.fdm;
    auto auxiliary = fdm.GetAuxiliary();
    auto propulsion = fdm.GetPropulsion();
    row.Set(Col::Mach, auxiliary->GetMach());
    row.Set(Col::Kcas, auxiliary->GetVcalibratedKTS());
    row.Set(Col::Alpha, auxiliary->Getalpha() * kRadToDeg);
    row.Set(Col::Beta, auxiliary->Getbeta() * kRadToDeg);
    row.Set(Col::Qbar, auxiliary->Getqbar());
    row.Set(Col::Nz, props.nz->getDoubleValue());
    row.Set(Col::Nx, props.nx->getDoubleValue());
    row.Set(Col::Ny, props.ny->getDoubleValue());
    row.Set(Col::RollRate, props.pRadSec->getDoubleValue() * kRadToDeg);
    row.Set(Col::PitchRate, props.qRadSec->getDoubleValue() * kRadToDeg);
    row.Set(Col::Weight, props.weightLbs->getDoubleValue());
    row.Set(Col::Fuel, propulsion->GetTanksWeight());
    double thrust = 0.0;
    for (unsigned int i = 0; i < propulsion->GetNumEngines(); ++i) thrust += propulsion->GetEngine(i)->GetThrust();
    row.Set(Col::Thrust, thrust);
    row.Set(Col::AileronCmd, props.aileronCmd->getDoubleValue());
    row.Set(Col::ElevatorCmd, props.elevatorCmd->getDoubleValue());
    row.Set(Col::ThrottleCmd, props.throttleCmd->getDoubleValue());
}

// Thrust by motor phase, the same schedule GuidanceSystem flies
// (REFERENCE.md, Missile propulsion).
double MissileThrustLbf(const GuidanceComponent& guidance, double timeSinceLaunch) {
    if (!guidance.launched || timeSinceLaunch < 0.0) return 0.0;
    if (timeSinceLaunch < guidance.boostDurationS) return guidance.boostThrustLbf;
    if (timeSinceLaunch < guidance.boostDurationS + guidance.sustainDurationS) return guidance.sustainThrustLbf;
    return 0.0;
}

void AddMissileColumns(Row& row, World& world, const SimContext& simContext, const KinematicsComponent& kin,
                       const Vec3& pos, double simTime, EntityID entity) {
    // Mach on the velocity through the air, as GuidanceSystem flies it.
    Vec3 airVel = kin.vel - WindFps(simContext.environment);
    Atmosphere::AtmosphereState air = Atmosphere::Lookup(pos.y(), simContext.environment.temperatureOffsetC);
    row.Set(Col::Mach, airVel.norm() / air.speedOfSoundFtPerSec);
    row.SetVec(Col::AccelX, kin.accel);

    const GuidanceComponent* guidance = world.Components<GuidanceComponent>().TryGet(entity);
    if (guidance == nullptr) return;
    double timeSinceLaunch = simTime - guidance->launchSimTime;
    row.Set(Col::Target, static_cast<unsigned int>(guidance->target));
    row.Set(Col::AchievedG, guidance->achievedAccel.norm() / kGravityFtPerSec2);
    row.Set(Col::MassSlug, GuidanceSystem::CurrentMassSlug(*guidance, guidance->launched ? timeSinceLaunch : 0.0));
    row.Set(Col::Thrust, MissileThrustLbf(*guidance, timeSinceLaunch));
    row.Set(Col::Launched, guidance->launched);
    row.Set(Col::Active, guidance->active);
    row.Set(Col::SeekerMode, SeekerModeName(guidance->seekerMode));
    row.Set(Col::SeekerActive, guidance->seekerActive);
    row.Set(Col::HasLock, guidance->hasLock);
    row.Set(Col::TimeSinceLockLost, guidance->timeSinceLockLostS);
    row.Set(Col::LaunchTurnDone, guidance->launchTurnDone);
    row.Set(Col::Lofting, guidance->lofting);

    const Transform* targetTransform = world.Components<Transform>().TryGet(guidance->target);
    if (targetTransform != nullptr) row.Set(Col::RangeToTarget, (targetTransform->pos - pos).norm());

    const DatalinkTrackComponent* datalink = world.Components<DatalinkTrackComponent>().TryGet(entity);
    if (datalink == nullptr || !datalink->hasTrack) return;
    row.Set(Col::EstAge, datalink->stalenessS);
    row.Set(Col::LinkLost, datalink->linkLost);
    row.SetVec(Col::EstX, datalink->estimatedPos);
    row.SetVec(Col::EstVelX, datalink->estimatedVel);
    if (targetTransform != nullptr) row.Set(Col::EstError, (datalink->estimatedPos - targetTransform->pos).norm());
}

void AddSensorColumns(Row& row, World& world, EntityID entity) {
    if (const AutopilotComponent* autopilot = world.Components<AutopilotComponent>().TryGet(entity)) {
        row.Set(Col::ApHeading, autopilot->targetHeadingDeg);
        row.Set(Col::ApAltitude, autopilot->targetAltitudeFt);
        row.Set(Col::ApSpeed, autopilot->targetSpeedKts);
    }
    if (const GLimitComponent* gLimit = world.Components<GLimitComponent>().TryGet(entity)) {
        row.Set(Col::Damage, gLimit->cumulativeDamage);
    }
    if (const WaypointComponent* waypoints = world.Components<WaypointComponent>().TryGet(entity)) {
        row.Set(Col::WaypointsLeft, static_cast<unsigned int>(waypoints->queue.size()));
    }
    if (const AirborneRadarComponent* radar = world.Components<AirborneRadarComponent>().TryGet(entity)) {
        if (radar->sttTarget != kNoEntity) row.Set(Col::SttTarget, static_cast<unsigned int>(radar->sttTarget));
    }
    if (const RwrComponent* rwr = world.Components<RwrComponent>().TryGet(entity)) {
        row.Set(Col::RwrMode, RwrModeName(rwr->perceivedMode));
        row.Set(Col::RwrIlluminated, rwr->currentlyIlluminated);
        row.Set(Col::RwrMissileWarning, rwr->missileWarning);
    }
}

}  // namespace

bool TickRecorder::Open(const std::string& path, std::string& error) {
    Close();
    file_.open(path, std::ios::out | std::ios::trunc | std::ios::binary);
    if (!file_.is_open()) {
        error = "cannot write " + path;
        return false;
    }
    path_ = path;
    line_.clear();
    for (std::size_t i = 0; i < std::size(kHeaders); ++i) {
        if (i > 0) line_ += ',';
        line_ += kHeaders[i];
    }
    line_ += '\n';
    file_ << line_;
    return true;
}

void TickRecorder::Close() {
    if (file_.is_open()) file_.close();
    path_.clear();
}

void TickRecorder::Sample(World& world, const SimContext& simContext, double simTime) {
    if (!file_.is_open()) return;
    line_.clear();
    Row row;

    ComponentStore<Tag>& tags = world.Components<Tag>();
    ComponentStore<JSBSimComponent>& jsbsims = world.Components<JSBSimComponent>();
    ComponentStore<KinematicsComponent>& kinematics = world.Components<KinematicsComponent>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();
    for (auto& [entity, transform] : world.Components<Transform>()) {
        const JSBSimComponent* jsbsim = jsbsims.TryGet(entity);
        const KinematicsComponent* kin = kinematics.TryGet(entity);
        row.Set(Col::SimTime, simTime);
        row.Set(Col::Kind, jsbsim != nullptr ? "aircraft" : kin != nullptr ? "missile" : "other");
        row.Set(Col::Entity, static_cast<unsigned int>(entity));
        if (const Tag* tag = tags.TryGet(entity)) row.SetName(tag->name);
        row.Set(Col::Destroyed, destroyed.Has(entity));
        row.SetVec(Col::PosX, transform.pos);
        row.SetVec(Col::Roll, transform.euler);
        Vec3 vel = EntityVelocity::GetWithVelocityFallback(world, entity);
        row.SetVec(Col::VelX, vel);
        row.Set(Col::Speed, vel.norm());
        if (jsbsim != nullptr && jsbsim->fdm != nullptr) AddAircraftColumns(row, *jsbsim);
        if (kin != nullptr) AddMissileColumns(row, world, simContext, *kin, transform.pos, simTime, entity);
        AddSensorColumns(row, world, entity);
        row.AppendTo(line_);
    }

    ComponentStore<Transform>& transforms = world.Components<Transform>();
    for (const AirborneRadarTrack& track : simContext.airborneRadarTracks) {
        row.Set(Col::SimTime, simTime);
        row.Set(Col::Kind, "track");
        row.Set(Col::Entity, static_cast<unsigned int>(track.owner));
        if (const Tag* tag = tags.TryGet(track.owner)) row.SetName(tag->name);
        row.Set(Col::Target, static_cast<unsigned int>(track.target));
        row.Set(Col::TrackState, TrackStateName(track.state));
        row.Set(Col::FrameHits, HitWindow::Count(track.frameHits));
        row.Set(Col::MissedFrames, track.missedFrames);
        if (track.estimate.valid) {
            Vec3 estimatedPos = RadarTrackFilter::Position(track.estimate);
            row.Set(Col::EstAge, simTime - track.lastSeenTime);
            row.SetVec(Col::EstX, estimatedPos);
            row.SetVec(Col::EstVelX, RadarTrackFilter::Velocity(track.estimate));
            const std::array<double, 36>& p = track.estimate.covariance;
            row.Set(Col::EstPosSigma, std::sqrt(p[0] + p[7] + p[14]));
            if (const Transform* truth = transforms.TryGet(track.target)) {
                row.Set(Col::EstError, (estimatedPos - truth->pos).norm());
            }
        }
        row.AppendTo(line_);
    }
    file_ << line_;
}
