#pragma once

#include <cstdint>

#include "ecs/World.h"
#include "sim/ReceiverBudget.h"
#include "util/Numeric.h"
#include "util/Vec3.h"

// ARH is autonomous from launch. SARH additionally requires continuous
// illumination from the launching aircraft for its whole flight.
// MIDCOURSE_ARH is the two-phase combination: it flies the midcourse on
// the launcher's track like SARH, then latches autonomous once its own
// active seeker acquires. That latch is one-way. See REFERENCE.md -
// Seeker Modeling.
enum class SeekerMode { ARH, SARH, MIDCOURSE_ARH };

// How the launcher's radar holds the target for the shot. TWS: fire on the
// scanning radar's confirmed track, which the target's RWR reads as search.
// STT: lock the target in single-target track first -- a fresher track,
// but the target's RWR reads a lock and the radar sees nothing else. SARH
// always uses STT, whatever this says: it homes on the lock's
// illumination. See REFERENCE.md - Single-target track.
enum class LaunchMode { TWS, STT };

// Proportional-navigation state for one missile, held between ticks.
struct GuidanceComponent {
    EntityID target = 0;
    double navGain = 3.0;
    Vec3 prevLosUnit;
    bool active = false;

    // Set permanently the instant this missile fires, distinct from
    // `active` (which also goes false once it resolves). The missile
    // entity exists before launch for inspectability, so `active` alone
    // cannot tell "never launched" from "launched and resolved".
    bool launched = false;

    // When the launch gate (a confirmed launcher track inside the launch
    // zone) last opened; negative while it is shut. The missile fires once
    // it has stayed open for the launcher radar's fire-control delay.
    double launchGateOpenTime = -1.0;
    LaunchMode launchMode = LaunchMode::TWS;

    // Closest-approach bookkeeping for InterceptSystem, living here since
    // GuidanceComponent is the missile's only state-carrying component.
    // Three consecutive range samples are kept, because closest approach
    // is detected one tick after it happens -- at that moment the three
    // bracket the true minimum, which is what the sub-tick fit needs.
    // Seeded at launch so the first tick sees a real, small delta.
    double prevRangeFt = 0.0;
    double prevPrevRangeFt = 0.0;
    double launchSimTime = 0.0;

    // The missile's position in the TARGET's body frame, at the same ticks
    // the ranges above were measured, so the reported impact point
    // describes the same instant as the reported miss distance rather than
    // wherever the missile has moved to by resolution time.
    Vec3 prevImpactBodyFt;
    Vec3 prevPrevImpactBodyFt;

    // Closest-approach confirmation. The candidate exists because
    // confirmation and measurement want different instants: the
    // interpolated CPA can only be computed on the FIRST opening tick,
    // while the three samples still bracket the minimum, but acting on it
    // is only sound a few ticks later.
    int cpaOpeningTicks = 0;
    double cpaCandidateRangeFt = 0.0;
    Vec3 cpaCandidateBodyFt;
    // Where the missile's path first entered the target's airframe box, in
    // the target's body frame, once it has. A pass straight through reads
    // zero miss distance all the way across, so the closest approach alone
    // cannot say where it struck; this can.
    bool impactEntryValid = false;
    Vec3 impactEntryBodyFt;
    // Whether the range has shrunk on any tick since launch. Opening range
    // is only a pass once the missile has been closing: a missile that
    // leaves the rail slower than a fleeing target opens the range until
    // its motor catches it up.
    bool rangeHasClosed = false;

    // The physical parameters below (motor, mass, drag area, seeker
    // hardware, fuze) are the missile type's, possibly overridden by the
    // scenario. They are zero here, not a default missile: a missile is
    // built from its type (MissileEntity::MakeGuidance, or LaunchSystem
    // from the scenario's doctrine).

    // Three-stage boost/sustain/coast thrust, force-based (lbf) so a
    // time-varying mass feeds through the same F=ma. See REFERENCE.md -
    // Three-stage propulsion.
    double boostDurationS = 0.0;
    double boostThrustLbf = 0.0;
    double sustainDurationS = 0.0;
    double sustainThrustLbf = 0.0;

    // Seeker gimbal cone with a brief memory-coast window before lock is
    // lost entirely -- a failure mode for aggressive maneuvering to
    // threaten, not a replacement for the guidance law.
    double seekerFovHalfAngleDeg = 0.0;
    double memoryCoastTimeS = 0.0;
    double timeSinceLockLostS = 0.0;
    bool hasLock = true;

    // Last real in-cone LOS rate, deliberately frozen while coasting on
    // memory rather than recomputed from a target the seeker cannot see.
    // prevLosUnit above keeps updating regardless; this does not.
    Vec3 lastLosRate;

    // Drag reference area (cross-sectional, distinct from radar cross-
    // section).
    double referenceAreaFt2 = 0.0;

    // Proximity-fuze arming interlock: how far the missile must fly from
    // its own launch point before a kill-radius pass can register at all.
    // Straight-line from launchPos, not accumulated path length -- a
    // documented simplification, fine near launch.
    double armingRangeFt = 0.0;
    Vec3 launchPos;

    // Self-destruct limits (MissileSelfDestructSystem): the available-G
    // floor after burnout, and the time-of-flight backstop. 0 disables
    // either, the default for a missile with no type.
    double selfDestructMinAvailableG = 0.0;
    double selfDestructMaxTimeOfFlightS = 0.0;

    // The guidance response time: one first-order lag between the
    // demanded and the delivered lateral acceleration, standing for the
    // seeker, fins and airframe together (sim/ResponseLag.h). 0 is
    // instantaneous. achievedAccel is the lagged lift, starting at zero:
    // fins neutral, flying straight off the rail.
    double responseTimeS = 0.0;
    Vec3 achievedAccel;

    // The launch turn: until the target first comes within
    // launchTurnExitDeg of the velocity vector, the autopilot turns toward
    // the line of sight at its available G instead of flying the guidance
    // law. 0 here (no turn) like every other value above; the type sets it.
    double launchTurnExitDeg = 0.0;
    bool launchTurnDone = false;

    // The loft (MissileLoft): the climb angle chosen at firing from the
    // launch range, 0 for no loft; lofting while it is being flown. The
    // climb ends for good once the target lies loftClimbDeg below the
    // missile's horizon (or, burnt out, once the G it has left falls under
    // loftMinAvailableG), and the launch turn then pushes it over.
    double loftClimbDeg = 0.0;
    double loftClimbTimeConstantS = 0.0;
    double loftMinAvailableG = 0.0;
    bool lofting = false;

    // Propellant depletes linearly over boost+sustain, so both thrust
    // acceleration and drag deceleration change over powered flight. With
    // the thrust profile this sets specific impulse, which is why the type
    // file constrains the two together. See REFERENCE.md - Mass depletion.
    double dryMassSlug = 0.0;
    double propellantMassSlug = 0.0;

    // `launcher` is whose AirborneRadarTrack SARH and midcourse
    // MIDCOURSE_ARH check illumination on; ARH ignores it.
    //
    // The mode is the type's (the AIM-120's is MIDCOURSE_ARH). ARH here
    // only for a missile built without one: it needs no launcher.
    SeekerMode seekerMode = SeekerMode::ARH;
    EntityID launcher = 0;

    // The missile's own active radar seeker, consulted only by
    // MIDCOURSE_ARH. Detects by SNR through the same radar equation, RCS
    // jitter, and aspect adjustment as every other detection here, rather
    // than a hardcoded "goes active at N miles". The receiver budget's
    // terms, then the SNR one sweep needs. See REFERENCE.md - The active
    // seeker.
    double seekerAveragePowerW = 0.0;
    double seekerAntennaGain = 0.0;
    double seekerFrequencyHz = 0.0;
    double seekerNoiseFigureDb = 0.0;
    double seekerIntegrationTimeS = 0.0;
    double seekerSystemLossDb = 0.0;
    double seekerThresholdDb = 0.0;
    // The teaching knob: where the seeker acquires the 5 m2 reference,
    // against the type's fitted range (see ReferenceRangeGainDb).
    double seekerReferenceRangeNmi = 0.0;
    double seekerTypeReferenceRangeNmi = 0.0;

    // The seeker sweeps at its own rate rather than every physics tick,
    // keeping RNG draws bounded and deterministic.
    double seekerSweepPeriodS = 0.0;
    double seekerLastSweepTime = Numeric::kLongAgoS;

    // Rolling M-of-N acquisition window, one bit per sweep (LSB = most
    // recent), bit-packed since this is the missile's only state-carrying
    // component. Not optional polish: under per-sweep RCS scintillation a
    // bare threshold test latches on the first favorable spike rather than
    // at the seeker's real range. See REFERENCE.md - The active seeker.
    uint32_t seekerSweepHistoryBits = 0;
    // Acquisition needs seekerConfirmHits detections in the last
    // seekerConfirmWindow sweeps; the type sets them (AIM-120: 3 of 5).
    // 1 of 1, a bare threshold test, for a missile built without a type.
    int seekerConfirmHits = 1;
    int seekerConfirmWindow = 1;

    // The active seeker's Doppler notch (the type's; see MissileSeeker).
    // A threshold of 0 never notches -- a missile built without a type.
    double seekerAntennaDiameterM = 0.0;
    double seekerBeamwidthFactor = 0.0;
    double seekerNotchVelocityThresholdMps = 0.0;

    // Set when the active seeker acquires. Cleared only by the datalink
    // fallback: a seeker that loses the target past its memory coast while
    // the launcher still tracks it hands back to datalink midcourse and
    // must acquire again (GuidanceSystem).
    bool seekerActive = false;

    // Set on the tick the seeker acquires (or hands back), consumed on the
    // next one. Guidance switches between steering on the datalinked track
    // and on the seeker's measurement at that moment, so the LOS vector
    // jumps by the track's error. Without this the finite-difference LOS
    // rate reads that jump as a real, enormous angular rate.
    bool navHandoffPending = false;
};

// Whether this missile still steers on the launcher's datalinked track.
// A MIDCOURSE_ARH missile flies the midcourse on it; once its own seeker
// acquires, it measures the target itself. ARH and SARH measure directly
// for their whole flight, so it is never true for them.
inline bool NavigatesInertially(const GuidanceComponent& guidance) {
    return guidance.seekerMode == SeekerMode::MIDCOURSE_ARH && !guidance.seekerActive;
}

// Whether this missile's own radar seeker is transmitting at its target:
// an ARH from launch, a MIDCOURSE_ARH once its seeker has gone active,
// until it hands back or the missile resolves. A SARH seeker only listens.
// What the target's RWR hears as a missile warning (RwrSystem).
inline bool SeekerRadiating(const GuidanceComponent& guidance) {
    if (!guidance.launched || !guidance.active) return false;
    return guidance.seekerMode == SeekerMode::ARH ||
           (guidance.seekerMode == SeekerMode::MIDCOURSE_ARH && guidance.seekerActive);
}

// Whether this missile's shot is taken from a single-target-track lock.
inline bool UsesStt(const GuidanceComponent& guidance) {
    return guidance.seekerMode == SeekerMode::SARH || guidance.launchMode == LaunchMode::STT;
}

// Whether fire control should hold the launcher's lock on this missile's
// target right now: before launch for any STT shot; after it while the
// missile still depends on the launcher -- SARH for its whole flight,
// MIDCOURSE_ARH until its own seeker goes active, ARH not at all.
inline bool NeedsLauncherLock(const GuidanceComponent& guidance) {
    if (!UsesStt(guidance)) return false;
    if (!guidance.launched) return true;
    if (!guidance.active) return false;  // resolved
    return guidance.seekerMode == SeekerMode::SARH || NavigatesInertially(guidance);
}

// The active seeker's receiver budget.
inline ReceiverBudget SeekerReceiverOf(const GuidanceComponent& guidance) {
    return ReceiverBudget{guidance.seekerAveragePowerW,    guidance.seekerAntennaGain,
                          WavelengthM(guidance.seekerFrequencyHz), guidance.seekerNoiseFigureDb,
                          guidance.seekerIntegrationTimeS,
                          guidance.seekerSystemLossDb - ReferenceRangeGainDb(guidance.seekerReferenceRangeNmi,
                                                                             guidance.seekerTypeReferenceRangeNmi)};
}
