#include "aircraft/AircraftCatalog.h"

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>

#include "sim/RadarEquation.h"
#include "util/TypeFileJson.h"

FluctuationModel ParseFluctuationModel(const std::string& name, const std::string& where) {
    if (name == "swerling_1") return FluctuationModel::SWERLING_1;
    if (name == "steady") return FluctuationModel::STEADY;
    throw std::runtime_error(where + ": fluctuation_model must be \"swerling_1\" or \"steady\", got \"" + name + "\"");
}

namespace {

// The confirmation window is held as bits of AirborneRadarTrack::frameHits
// (16 bits).
constexpr int kMaxConfirmWindowFrames = 15;

using TypeFileJson::RequireNumber;
using TypeFileJson::RequireObject;
using TypeFileJson::RequireString;

AircraftType Parse(const std::filesystem::path& file, const std::string& id, const std::filesystem::path& directory) {
    const std::string where = file.string();
    const nlohmann::json j = TypeFileJson::Read(file, "AircraftCatalog");
    AircraftType type;
    type.id = id;
    type.directory = directory;
    type.displayName = RequireString(j, "name", where);
    type.jsbsimModel = RequireString(j, "jsbsim_model", where);

    const nlohmann::json& geometry = RequireObject(j, "geometry", where);
    type.wingAreaFt2 = RequireNumber(geometry, "wing_area_ft2", where + " geometry");
    // Authored as whole dimensions, the way specifications quote them.
    const nlohmann::json& box = RequireObject(geometry, "hit_box_ft", where + " geometry");
    type.hitBox.halfSpanFt = 0.5 * RequireNumber(box, "span", where + " hit_box_ft");
    type.hitBox.halfHeightFt = 0.5 * RequireNumber(box, "height", where + " hit_box_ft");
    type.hitBox.halfLengthFt = 0.5 * RequireNumber(box, "length", where + " hit_box_ft");

    const nlohmann::json& envelope = RequireObject(j, "envelope", where);
    const std::string envelopeWhere = where + " envelope";
    type.envelope.clMax = RequireNumber(envelope, "cl_max", envelopeWhere);
    type.envelope.clMaxLossPerG = RequireNumber(envelope, "cl_max_loss_per_g", envelopeWhere);
    type.envelope.structuralGLimitPos = RequireNumber(envelope, "structural_g_limit_pos", envelopeWhere);
    type.envelope.structuralGLimitNeg = RequireNumber(envelope, "structural_g_limit_neg", envelopeWhere);
    type.envelope.damageAccumulationRate = RequireNumber(envelope, "damage_accumulation_rate", envelopeWhere);

    const nlohmann::json& controls = RequireObject(j, "flight_controls", where);
    const std::string controlsWhere = where + " flight_controls";
    type.controls.gPerUnitStick = RequireNumber(controls, "g_per_unit_stick", controlsWhere);
    type.controls.militaryThrottleCmd = RequireNumber(controls, "military_throttle_cmd", controlsWhere);
    type.controls.maxThrottleCmd = RequireNumber(controls, "max_throttle_cmd", controlsWhere);

    const nlohmann::json& fuel = RequireObject(j, "fuel", where);
    type.internalTankPrefix = RequireString(fuel, "internal_tank_prefix", where + " fuel");

    // Optional: a type with no fire-control radar omits it.
    if (j.contains("radar")) {
        const nlohmann::json& radar = RequireObject(j, "radar", where);
        const std::string radarWhere = where + " radar";
        AirborneRadarType& r = type.radar;
        r.present = true;
        r.frameTimeS = RequireNumber(radar, "frame_time_s", radarWhere);
        r.sttFrameTimeS = RequireNumber(radar, "stt_frame_time_s", radarWhere);
        r.averagePowerW = RequireNumber(radar, "average_power_w", radarWhere);
        r.peakPowerW = RequireNumber(radar, "peak_power_w", radarWhere);
        r.antennaGain = RequireNumber(radar, "antenna_gain", radarWhere);
        r.frequencyHz = RequireNumber(radar, "frequency_hz", radarWhere);
        r.noiseFigureDb = RequireNumber(radar, "noise_figure_db", radarWhere);
        r.timeOnTargetS = RequireNumber(radar, "time_on_target_s", radarWhere);
        r.systemLossDb = RequireNumber(radar, "system_loss_db", radarWhere);
        r.pfa = RequireNumber(radar, "pfa", radarWhere);
        // Noise alone crosses the search threshold with probability pfa.
        r.searchThresholdDb = r.pfa > 0.0 && r.pfa < 1.0 ? RadarEquation::DetectionThresholdDb(r.pfa) : 0.0;
        // Whole frames.
        double confirmFrames = RequireNumber(radar, "confirm_frames", radarWhere);
        double confirmWindow = RequireNumber(radar, "confirm_window_frames", radarWhere);
        double dropFrames = RequireNumber(radar, "drop_frames", radarWhere);
        double sttDropFrames = RequireNumber(radar, "stt_drop_frames", radarWhere);
        // Checked as doubles: casting an out-of-range one to int is undefined.
        auto wholeFrames = [](double v) { return std::isfinite(v) && std::fabs(v) < 1e6 && v == std::floor(v); };
        if (!wholeFrames(confirmFrames) || !wholeFrames(confirmWindow) || !wholeFrames(dropFrames) ||
            !wholeFrames(sttDropFrames) || confirmWindow < 1 || confirmWindow > kMaxConfirmWindowFrames || confirmFrames < 1 ||
            confirmFrames > confirmWindow || dropFrames < 1 || sttDropFrames < 1) {
            throw std::runtime_error(radarWhere + ": confirm_frames, confirm_window_frames, drop_frames and "
                                                  "stt_drop_frames must be whole numbers, 1 <= confirm_frames <= "
                                                  "confirm_window_frames <= " +
                                     std::to_string(kMaxConfirmWindowFrames) + " and both drops >= 1");
        }
        r.confirmFrames = static_cast<int>(confirmFrames);
        r.confirmWindowFrames = static_cast<int>(confirmWindow);
        r.dropFrames = static_cast<int>(dropFrames);
        r.sttDropFrames = static_cast<int>(sttDropFrames);
        r.fireControlDelayS = RequireNumber(radar, "fire_control_delay_s", radarWhere);
        r.fovHalfAngleDeg = RequireNumber(radar, "fov_half_angle_deg", radarWhere);
        r.notchVelocityThresholdMps = RequireNumber(radar, "notch_velocity_threshold_mps", radarWhere);
        r.antennaHeightM = RequireNumber(radar, "antenna_height_m", radarWhere);
        r.minRangeM = RequireNumber(radar, "min_range_m", radarWhere);
        r.antennaWidthM = RequireNumber(radar, "antenna_width_m", radarWhere);
        r.beamwidthFactor = RequireNumber(radar, "beamwidth_factor", radarWhere);
        r.monopulseSlope = RequireNumber(radar, "monopulse_slope", radarWhere);
        r.pulseWidthS = RequireNumber(radar, "pulse_width_s", radarWhere);
        r.trackManeuverG = RequireNumber(radar, "track_maneuver_g", radarWhere);
        if (r.sttFrameTimeS <= 0.0 || r.sttFrameTimeS > r.frameTimeS) {
            throw std::runtime_error(radarWhere + ": stt_frame_time_s must be positive and no longer than "
                                                  "frame_time_s: a lock looks at its one target at least as "
                                                  "often as a scan does");
        }
        if (r.frameTimeS <= 0.0 || r.averagePowerW <= 0.0 || r.peakPowerW < r.averagePowerW ||
            r.antennaGain <= 0.0 || r.frequencyHz <= 0.0 || r.timeOnTargetS <= 0.0 || !(r.pfa > 0.0 && r.pfa < 1.0) ||
            r.fireControlDelayS < 0.0 || r.fovHalfAngleDeg <= 0.0 ||
            r.antennaHeightM <= 0.0 || r.antennaWidthM <= 0.0 ||
            r.beamwidthFactor <= 0.0 || r.monopulseSlope <= 0.0 || r.pulseWidthS <= 0.0 || r.trackManeuverG < 0.0) {
            throw std::runtime_error(radarWhere + ": a value is out of its physical range (powers, gain, frequency, "
                                                  "frame time, time on target, antenna size, beamwidth factor, "
                                                  "monopulse slope and pulse width "
                                                  "positive; track maneuver g and fire-control delay not negative; "
                                                  "peak power at least the average; pfa between 0 and 1)");
        }
    }

    // The range the radar's system loss was fitted to, if the type has a
    // fit (tools/fit/radar_budget.py writes the file it reads).
    if (type.radar.present) {
        const std::filesystem::path fitFile = directory / "fit" / "radar_fit.json";
        if (std::filesystem::exists(fitFile)) {
            type.radar.referenceRangeNmi =
                RequireNumber(TypeFileJson::Read(fitFile, "AircraftCatalog"), "range_nmi", fitFile.string());
        }
    }

    // Optional: a type with no radar warning receiver omits it.
    if (j.contains("rwr")) {
        const nlohmann::json& rwr = RequireObject(j, "rwr", where);
        const std::string rwrWhere = where + " rwr";
        type.rwr.present = true;
        type.rwr.sensitivityDbm = RequireNumber(rwr, "sensitivity_dbm", rwrWhere);
        type.rwr.antennaGain = RequireNumber(rwr, "antenna_gain", rwrWhere);
        type.rwr.signalJitterDb = RequireNumber(rwr, "signal_jitter_db", rwrWhere);
        type.rwr.sweepPeriodS = RequireNumber(rwr, "sweep_period_s", rwrWhere);
        if (type.rwr.antennaGain <= 0.0) {
            throw std::runtime_error(rwrWhere + ": antenna_gain must be positive (linear, not dB)");
        }
        if (!(type.rwr.sweepPeriodS > 0.0)) {
            throw std::runtime_error(rwrWhere + ": sweep_period_s must be positive");
        }
        if (!(type.rwr.signalJitterDb >= 0.0)) {
            throw std::runtime_error(rwrWhere + ": signal_jitter_db must not be negative");
        }
    }

    // Optional: a type with no signature is invisible to radar.
    if (j.contains("signature")) {
        const nlohmann::json& signature = RequireObject(j, "signature", where);
        const std::string signatureWhere = where + " signature";
        type.signature.present = true;
        if (signature.contains("static_rcs_m2")) {
            throw std::runtime_error(signatureWhere + ": static_rcs_m2 is now derived from rcs_shape (the nose-on "
                                                      "projected area; tools/fit/rcs_shape.py)");
        }
        if (!signature.contains("rcs_shape") || !signature.at("rcs_shape").is_array() ||
            signature.at("rcs_shape").empty() || signature.at("rcs_shape").size() > kMaxRcsEllipsoids) {
            throw std::runtime_error(signatureWhere + ": rcs_shape must be an array of 1-" +
                                     std::to_string(kMaxRcsEllipsoids) + " ellipsoids");
        }
        for (const nlohmann::json& part : signature.at("rcs_shape")) {
            const std::string partWhere = signatureWhere + " rcs_shape";
            const nlohmann::json& axes = RequireObject(part, "semi_axes_ft", partWhere);
            RcsEllipsoid& e = type.signature.shape[type.signature.shapeCount++];
            e.spanFt = RequireNumber(axes, "span", partWhere);
            e.heightFt = RequireNumber(axes, "height", partWhere);
            e.lengthFt = RequireNumber(axes, "length", partWhere);
            if (e.spanFt <= 0.0 || e.heightFt <= 0.0 || e.lengthFt <= 0.0) {
                throw std::runtime_error(partWhere + ": every semi-axis must be positive");
            }
        }
        type.signature.staticRcsM2 =
            RadarEquation::ShapeProjectedAreaM2(type.signature.shape.data(), type.signature.shapeCount,
                                                Vec3{0.0, 0.0, 1.0});
        type.signature.fluctuation = ParseFluctuationModel(
            TypeFileJson::RequireString(signature, "fluctuation_model", signatureWhere), signatureWhere);
    }

    if (type.hitBox.halfSpanFt <= 0.0 || type.hitBox.halfHeightFt <= 0.0 || type.hitBox.halfLengthFt <= 0.0) {
        throw std::runtime_error(where + " geometry hit_box_ft: span, height and length must be positive");
    }
    if (type.internalTankPrefix.empty()) {
        throw std::runtime_error(where + " fuel: internal_tank_prefix must not be empty (an empty prefix matches "
                                         "every tank, drop tanks included)");
    }
    if (!(type.envelope.damageAccumulationRate >= 0.0)) {
        throw std::runtime_error(where + " envelope: damage_accumulation_rate must not be negative");
    }
    if (type.wingAreaFt2 <= 0.0 || type.envelope.clMax <= 0.0 || type.envelope.clMaxLossPerG < 0.0 ||
        type.envelope.structuralGLimitPos <= 1.0 ||
        type.envelope.structuralGLimitNeg >= 0.0 || type.controls.gPerUnitStick <= 0.0 ||
        type.controls.maxThrottleCmd <= 0.0) {
        throw std::runtime_error(where + ": a value is out of its physical range (wing area, cl_max and "
                                         "g_per_unit_stick positive; +G limit above 1; -G limit negative)");
    }
    return type;
}

}  // namespace

AircraftCatalog::AircraftCatalog(std::filesystem::path root)
    : TypeCatalog(std::move(root), "aircraft.json", "AircraftCatalog", "aircraft type", &Parse) {}

AircraftCatalog& AircraftCatalog::Default() {
    static AircraftCatalog catalog("data/aircraft");
    return catalog;
}
