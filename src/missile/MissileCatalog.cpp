#include "missile/MissileCatalog.h"

#include <cstdio>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sim/RadarEquation.h"
#include "sim/TableLookup.h"
#include "util/TypeFileJson.h"
#include "util/Units.h"

namespace {

// The acquisition window is held as bits of
// GuidanceComponent::seekerSweepHistoryBits (32 bits).
constexpr int kMaxConfirmWindowSweeps = 31;

using TypeFileJson::RequireNumber;
using TypeFileJson::RequireObject;
using TypeFileJson::RequireString;

SeekerMode ParseSeekerMode(const std::string& name, const std::string& where) {
    if (name == "ARH") return SeekerMode::ARH;
    if (name == "SARH") return SeekerMode::SARH;
    if (name == "MIDCOURSE_ARH") return SeekerMode::MIDCOURSE_ARH;
    throw std::runtime_error(where + ": \"mode\" must be \"ARH\", \"SARH\" or \"MIDCOURSE_ARH\", got: " + name);
}

// Authored as [mach, cd] rows. Strictly increasing Mach and positive Cd,
// so the interpolation is well defined everywhere.
DragTable ParseDragTable(const nlohmann::json& aero, const std::string& where) {
    if (!aero.contains("cd_vs_mach") || !aero.at("cd_vs_mach").is_array() || aero.at("cd_vs_mach").empty()) {
        throw std::runtime_error(where + ": missing or empty \"cd_vs_mach\" table");
    }
    std::vector<DragTable::Point> points;
    for (const nlohmann::json& row : aero.at("cd_vs_mach")) {
        if (!row.is_array() || row.size() != 2 || !row.at(0).is_number() || !row.at(1).is_number()) {
            throw std::runtime_error(where + ": every \"cd_vs_mach\" row must be [mach, cd]");
        }
        DragTable::Point point{row.at(0).get<double>(), row.at(1).get<double>()};
        if (point.cd <= 0.0 || (!points.empty() && point.mach <= points.back().mach)) {
            throw std::runtime_error(where + ": \"cd_vs_mach\" needs positive cd in strictly increasing mach");
        }
        points.push_back(point);
    }
    return DragTable(std::move(points));
}

// [key, value] rows in strictly increasing key with positive values.
std::vector<std::pair<double, double>> ParseRows(const nlohmann::json& block, const char* key, const std::string& where) {
    if (!block.contains(key) || !block.at(key).is_array() || block.at(key).empty()) {
        throw std::runtime_error(where + ": missing or empty \"" + key + "\" table");
    }
    std::vector<std::pair<double, double>> rows;
    for (const nlohmann::json& row : block.at(key)) {
        if (!row.is_array() || row.size() != 2 || !row.at(0).is_number() || !row.at(1).is_number()) {
            throw std::runtime_error(where + ": every \"" + key + "\" row must be [key, value]");
        }
        std::pair<double, double> point{row.at(0).get<double>(), row.at(1).get<double>()};
        if (point.second <= 0.0 || (!rows.empty() && point.first <= rows.back().first)) {
            throw std::runtime_error(where + ": \"" + key + "\" needs positive values in strictly increasing keys");
        }
        rows.push_back(point);
    }
    return rows;
}

// aerodynamics.normal_force: alpha_max and the two tables over Mach; see
// MissileNormalForce.h.
std::shared_ptr<const MissileNormalForce> ParseNormalForce(const nlohmann::json& aero, double referenceAreaFt2,
                                                           const std::string& where) {
    const nlohmann::json& normalForce = RequireObject(aero, "normal_force", where);
    const std::string nfWhere = where + " normal_force";
    double alphaMaxDeg = RequireNumber(normalForce, "alpha_max_deg", nfWhere);
    if (alphaMaxDeg <= 0.0 || alphaMaxDeg > 90.0) {
        throw std::runtime_error(nfWhere + ": \"alpha_max_deg\" must be in (0, 90]");
    }
    MissileNormalForce::Table slope = ParseRows(normalForce, "cn_alpha_per_deg_vs_mach", nfWhere);
    MissileNormalForce::Table cnMax = ParseRows(normalForce, "cn_max_vs_mach", nfWhere);
    // CN = a alpha + b alpha^2 rises on [0, alpha_max] only if b >= 0. b is
    // linear between the Mach breakpoints of the two tables, so checking at
    // every breakpoint of either covers every Mach.
    using Row = MissileNormalForce::Table::value_type;
    auto check = [&](double mach) {
        if (TableLookup::Lookup(cnMax, mach, &Row::first, &Row::second) <
            TableLookup::Lookup(slope, mach, &Row::first, &Row::second) * alphaMaxDeg) {
            throw std::runtime_error(nfWhere + ": \"cn_max_vs_mach\" must be at least CN_alpha * alpha_max at "
                                               "every Mach (at Mach " + std::to_string(mach) + " it is less)");
        }
    };
    for (const auto& row : slope) check(row.first);
    for (const auto& row : cnMax) check(row.first);
    return std::make_shared<const MissileNormalForce>(std::move(slope), std::move(cnMax), alphaMaxDeg,
                                                      referenceAreaFt2);
}

MissileType Parse(const std::filesystem::path& file, const std::string& id, const std::filesystem::path& directory) {
    const std::string where = file.string();
    const nlohmann::json j = TypeFileJson::Read(file, "MissileCatalog");
    MissileType type;
    type.id = id;
    type.directory = directory;
    type.displayName = RequireString(j, "name", where);

    const nlohmann::json& propulsion = RequireObject(j, "propulsion", where);
    const std::string propulsionWhere = where + " propulsion";
    type.propulsion.boostDurationS = RequireNumber(propulsion, "boost_duration_s", propulsionWhere);
    type.propulsion.boostThrustLbf = RequireNumber(propulsion, "boost_thrust_lbf", propulsionWhere);
    type.propulsion.sustainDurationS = RequireNumber(propulsion, "sustain_duration_s", propulsionWhere);
    type.propulsion.sustainThrustLbf = RequireNumber(propulsion, "sustain_thrust_lbf", propulsionWhere);

    const nlohmann::json& mass = RequireObject(j, "mass", where);
    type.mass.dryMassSlug = RequireNumber(mass, "dry_mass_slug", where + " mass");
    type.mass.propellantMassSlug = RequireNumber(mass, "propellant_mass_slug", where + " mass");

    const nlohmann::json& aero = RequireObject(j, "aerodynamics", where);
    const std::string aeroWhere = where + " aerodynamics";
    type.aero.referenceAreaFt2 = RequireNumber(aero, "reference_area_ft2", aeroWhere);
    type.aero.drag = ParseDragTable(aero, aeroWhere);
    type.aero.normalForce = ParseNormalForce(aero, type.aero.referenceAreaFt2, aeroWhere);
    type.aero.structuralGLimit = RequireNumber(aero, "structural_g_limit", aeroWhere);

    const nlohmann::json& seeker = RequireObject(j, "seeker", where);
    const std::string seekerWhere = where + " seeker";
    type.seeker.mode = ParseSeekerMode(RequireString(seeker, "mode", seekerWhere), seekerWhere);
    type.seeker.fovHalfAngleDeg = RequireNumber(seeker, "fov_half_angle_deg", seekerWhere);
    type.seeker.memoryCoastTimeS = RequireNumber(seeker, "memory_coast_time_s", seekerWhere);
    const nlohmann::json& active = RequireObject(seeker, "active_seeker", seekerWhere);
    const std::string activeWhere = seekerWhere + " active_seeker";
    type.seeker.averagePowerW = RequireNumber(active, "average_power_w", activeWhere);
    type.seeker.antennaGain = RequireNumber(active, "antenna_gain", activeWhere);
    type.seeker.frequencyHz = RequireNumber(active, "frequency_hz", activeWhere);
    type.seeker.noiseFigureDb = RequireNumber(active, "noise_figure_db", activeWhere);
    type.seeker.integrationTimeS = RequireNumber(active, "integration_time_s", activeWhere);
    type.seeker.systemLossDb = RequireNumber(active, "system_loss_db", activeWhere);
    type.seeker.pfa = RequireNumber(active, "pfa", activeWhere);
    // Noise alone crosses it with probability pfa.
    type.seeker.thresholdDb =
        type.seeker.pfa > 0.0 && type.seeker.pfa < 1.0 ? RadarEquation::DetectionThresholdDb(type.seeker.pfa) : 0.0;
    type.seeker.sweepPeriodS = RequireNumber(active, "sweep_period_s", activeWhere);
    // Whole sweeps.
    double confirmHits = RequireNumber(active, "confirm_hits", activeWhere);
    double confirmWindow = RequireNumber(active, "confirm_window", activeWhere);
    if (confirmHits != static_cast<int>(confirmHits) || confirmWindow != static_cast<int>(confirmWindow) ||
        confirmWindow < 1 || confirmWindow > kMaxConfirmWindowSweeps || confirmHits < 1 ||
        confirmHits > confirmWindow) {
        throw std::runtime_error(activeWhere + ": \"confirm_window\" must be a whole number 1-" +
                                 std::to_string(kMaxConfirmWindowSweeps) +
                                 " and \"confirm_hits\" a whole number from 1 to the window");
    }
    type.seeker.confirmHits = static_cast<int>(confirmHits);
    type.seeker.confirmWindow = static_cast<int>(confirmWindow);
    type.seeker.antennaDiameterM = RequireNumber(active, "antenna_diameter_m", activeWhere);
    type.seeker.beamwidthFactor = RequireNumber(active, "beamwidth_factor", activeWhere);
    type.seeker.notchVelocityThresholdMps = RequireNumber(active, "notch_velocity_threshold_mps", activeWhere);
    if (type.seeker.antennaDiameterM <= 0.0 || type.seeker.beamwidthFactor <= 0.0 ||
        type.seeker.notchVelocityThresholdMps < 0.0) {
        throw std::runtime_error(activeWhere + ": \"antenna_diameter_m\" and \"beamwidth_factor\" must be positive "
                                               "and \"notch_velocity_threshold_mps\" not negative");
    }

    // Every missile flies true PN with the gravity bias on.
    const nlohmann::json& autopilot = RequireObject(j, "autopilot", where);
    type.autopilot.responseTimeS = RequireNumber(autopilot, "response_time_s", where + " autopilot");
    type.autopilot.launchTurnExitDeg = RequireNumber(autopilot, "launch_turn_exit_deg", where + " autopilot");
    if (type.autopilot.launchTurnExitDeg < 0.0 || type.autopilot.launchTurnExitDeg >= 180.0) {
        throw std::runtime_error(where + " autopilot: \"launch_turn_exit_deg\" must be in [0, 180) (0 = no launch turn)");
    }

    const nlohmann::json& loft = RequireObject(j, "loft", where);
    const std::string loftWhere = where + " loft";
    type.loft.startRangeNmi = RequireNumber(loft, "start_range_nmi", loftWhere);
    type.loft.fullRangeNmi = RequireNumber(loft, "full_range_nmi", loftWhere);
    type.loft.maxClimbDeg = RequireNumber(loft, "max_climb_deg", loftWhere);
    type.loft.climbTimeConstantS = RequireNumber(loft, "climb_time_constant_s", loftWhere);
    type.loft.minAvailableG = RequireNumber(loft, "min_available_g", loftWhere);
    if (type.loft.startRangeNmi < 0.0 || type.loft.fullRangeNmi <= type.loft.startRangeNmi ||
        type.loft.maxClimbDeg < 0.0 || type.loft.maxClimbDeg >= 90.0 || type.loft.climbTimeConstantS <= 0.0 ||
        type.loft.minAvailableG < 0.0) {
        throw std::runtime_error(loftWhere + ": need 0 <= \"start_range_nmi\" < \"full_range_nmi\", "
                                             "\"max_climb_deg\" in [0, 90) (0 = never lofts), a positive "
                                             "\"climb_time_constant_s\" and \"min_available_g\" not negative");
    }

    // Metres, like the doctrine's arming_range_m override.
    const nlohmann::json& fuze = RequireObject(j, "fuze", where);
    type.fuze.armingRangeFt = RequireNumber(fuze, "arming_range_m", where + " fuze") * Units::kFeetPerMeter;
    type.fuze.falloffWidthFt = RequireNumber(fuze, "falloff_width_m", where + " fuze") * Units::kFeetPerMeter;

    const nlohmann::json& selfDestruct = RequireObject(j, "self_destruct", where);
    const std::string selfDestructWhere = where + " self_destruct";
    type.selfDestruct.minAvailableG = RequireNumber(selfDestruct, "min_available_g", selfDestructWhere);
    type.selfDestruct.maxTimeOfFlightS = RequireNumber(selfDestruct, "max_time_of_flight_s", selfDestructWhere);
    // A floor at or above the structural cap would destroy every missile at
    // burnout; a time limit inside the burn would destroy it under power.
    if (type.selfDestruct.minAvailableG <= 0.0 || type.selfDestruct.minAvailableG >= type.aero.structuralGLimit) {
        throw std::runtime_error(selfDestructWhere + ": \"min_available_g\" must be above 0 and below "
                                                     "aerodynamics.structural_g_limit");
    }
    if (type.selfDestruct.maxTimeOfFlightS <= type.propulsion.boostDurationS + type.propulsion.sustainDurationS) {
        throw std::runtime_error(selfDestructWhere + ": \"max_time_of_flight_s\" must be longer than the motor burn");
    }

    const MissilePropulsion& p = type.propulsion;
    const MissileSeeker& s = type.seeker;
    if (p.boostDurationS < 0.0 || p.boostThrustLbf < 0.0 || p.sustainDurationS < 0.0 || p.sustainThrustLbf < 0.0 ||
        type.mass.dryMassSlug <= 0.0 || type.mass.propellantMassSlug < 0.0 || type.aero.referenceAreaFt2 <= 0.0 ||
        type.aero.structuralGLimit <= 0.0 ||
        s.fovHalfAngleDeg <= 0.0 || s.fovHalfAngleDeg > 180.0 || s.memoryCoastTimeS < 0.0 ||
        s.averagePowerW <= 0.0 || s.antennaGain <= 0.0 || s.frequencyHz <= 0.0 ||
        s.integrationTimeS <= 0.0 || !(s.pfa > 0.0 && s.pfa < 1.0) || s.sweepPeriodS <= 0.0 || type.fuze.armingRangeFt < 0.0 ||
        type.autopilot.responseTimeS < 0.0) {
        throw std::runtime_error(where + ": a value is out of its physical range (dry mass, reference area, "
                                         "G limit and the seeker's receiver budget positive, its pfa between 0 and "
                                         "1; "
                                         "durations, thrusts, propellant, arming range and response time not negative; "
                                         "seeker cone at most 180 deg)");
    }

    // The range the seeker's system loss was fitted to, if the type has a
    // fit (tools/fit/radar_budget.py writes the file it reads).
    const std::filesystem::path seekerFitFile = directory / "fit" / "radar_fit.json";
    if (std::filesystem::exists(seekerFitFile)) {
        type.seeker.referenceRangeNmi =
            RequireNumber(TypeFileJson::Read(seekerFitFile, "MissileCatalog"), "range_nmi", seekerFitFile.string());
    }

    // Generated from this file by tools/drivers/launch_zone.cpp, so it may not exist
    // yet -- the tool itself loads the type before there is one.
    const std::filesystem::path zoneFile = directory / "fit" / "launch_zone.json";
    if (std::filesystem::exists(zoneFile)) {
        type.launchZone = std::make_shared<const LaunchZone>(LaunchZone::Load(zoneFile));
        type.launchZoneStale = type.launchZone->MissileFileHash() != LaunchZone::FileHash(file);
        if (type.launchZoneStale) {
            std::fprintf(stderr,
                         "MissileCatalog: warning: %s was generated from a different %s -- rerun "
                         "tools/drivers/launch_zone.cpp\n",
                         zoneFile.string().c_str(), file.string().c_str());
        }
    }
    return type;
}

}  // namespace

MissileCatalog::MissileCatalog(std::filesystem::path root)
    : TypeCatalog(std::move(root), "missile.json", "MissileCatalog", "missile type", &Parse) {}

MissileCatalog& MissileCatalog::Default() {
    static MissileCatalog catalog("data/missiles");
    return catalog;
}
