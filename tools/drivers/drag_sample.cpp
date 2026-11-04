// 1 g drag sampler for fitting an aircraft type's engine thrust tables.
//
// Installed thrust is rarely published, but a flight manual's level
// accelerations measure thrust minus drag: in level flight
//
//   (W/g) dV/dt = T cos(alpha) - D
//
// so thrust follows once drag and alpha are known. Drag is the part of the
// model grounded in measured data, so this tool reads it off the model: at
// each altitude the type's reference tables cover, and each frozen gross
// weight its fit settings list, it flies a level acceleration at full
// power, the autopilot holding altitude, and records drag, alpha and load
// factor in 1 s windows across Mach. Drag does not depend on thrust, so
// runs that stop short of a table's top Mach are filled by re-running
// after the next thrust fit. tools/fit/thrust_fit.py does the fitting.
//
// Inputs, all under data/aircraft/<id>/:
//   reference/flight_manual_accel.csv  altitudes and Mach ranges to cover
//   reference/climb_envelope.csv       also its MIL rows' altitudes, when
//                                      fit_mil_to_climb_envelope is set
//   fit/thrust_fit.json                drag_sample_gross_weights_lb
// Output: fit/drag_1g.csv.
//
// Run from the repo root (data paths are relative to it):
//   build/Release/tools/drag_sample.exe --aircraft f16c_block50

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "FGFDMExec.h"

#include "aircraft/AircraftCatalog.h"
#include "sim/Atmosphere.h"
#include "systems/FlightDynamicsSystem.h"
#include "util/Units.h"
#include "validation/AircraftRig.h"

namespace {

// Mach margins around the reference data's range: start a little below its
// slowest point, run a little past its fastest.
constexpr double kStartMachMargin = 0.02;
constexpr double kEndMachMargin = 0.05;
constexpr double kEnvelopeMachHalfWidth = 0.06;  // a climb-envelope row's band
// Trim retries: up to this far above the band's start, in these steps.
constexpr double kTrimRetrySpanMach = 0.1;
constexpr double kTrimRetryStepMach = 0.02;
constexpr double kSettleS = 10.0;
constexpr double kFullPowerCmdKts = 3000.0;  // unreachable: full power throughout
constexpr int kMaxWindows = 900;              // 1 s each
// Top speed: Mach rose less than kPlateauMach over kPlateauWindows windows.
constexpr int kPlateauWindows = 10;
constexpr double kPlateauMach = 0.002;
// Sea-level tables are flown at 200 ft, clear of the terrain; the 0.6%
// density difference is ignored.
constexpr double kSeaLevelFlownFt = 200.0;

struct Band {
    double altitudeFt = 0.0;  // as flown
    double startMach = 0.0;
    double maxMach = 0.0;
};

// One band per altitude in the reference tables: from just below the
// slowest tabulated Mach to just past the fastest.
std::vector<Band> BandsFromReference(const std::filesystem::path& csvPath) {
    std::ifstream in(csvPath);
    if (!in) throw std::runtime_error("drag_sample: cannot read " + csvPath.string());
    std::string line;
    std::getline(in, line);  // header
    std::vector<std::string> columns;
    {
        std::stringstream header(line);
        std::string column;
        while (std::getline(header, column, ',')) columns.push_back(column);
    }
    auto index = [&](const char* name) {
        auto found = std::find(columns.begin(), columns.end(), name);
        if (found == columns.end()) throw std::runtime_error("drag_sample: no column " + std::string(name));
        return static_cast<size_t>(found - columns.begin());
    };
    size_t altitudeColumn = index("altitude_ft"), machColumn = index("mach");
    std::map<double, std::pair<double, double>> range;  // altitude -> (min, max) Mach
    while (std::getline(in, line)) {
        std::vector<std::string> cells;
        std::stringstream row(line);
        std::string cell;
        while (std::getline(row, cell, ',')) cells.push_back(cell);
        double altitude = std::stod(cells[altitudeColumn]);
        double mach = std::stod(cells[machColumn]);
        auto [it, inserted] = range.try_emplace(altitude, mach, mach);
        it->second.first = std::min(it->second.first, mach);
        it->second.second = std::max(it->second.second, mach);
    }
    std::vector<Band> bands;
    for (const auto& [altitude, machs] : range) {
        bands.push_back({altitude == 0.0 ? kSeaLevelFlownFt : altitude, machs.first - kStartMachMargin, machs.second + kEndMachMargin});
    }
    return bands;
}

// Bands at the climb-envelope altitudes the thrust fit uses for `power`
// (reference/climb_envelope.csv): one per row, around its Mach.
std::vector<Band> BandsFromEnvelope(const std::filesystem::path& csvPath, const std::string& power) {
    std::vector<Band> bands;
    std::ifstream in(csvPath);
    if (!in) return bands;
    std::string line;
    std::getline(in, line);  // power,gross_weight_lb,mach,climb_fpm,altitude_ft,...
    while (std::getline(in, line)) {
        std::stringstream row(line);
        std::vector<std::string> cells;
        std::string cell;
        while (std::getline(row, cell, ',')) cells.push_back(cell);
        if (cells.size() < 5 || cells[0] != power) continue;
        double mach = std::stod(cells[2]);
        bands.push_back({std::stod(cells[4]), mach - kEnvelopeMachHalfWidth, mach + kEnvelopeMachHalfWidth});
    }
    return bands;
}

// One level acceleration from startMach until speed stops rising or
// maxMach is passed; a row per 1 s window, each the window's average.
// False if no start Mach trims (too heavy for this altitude).
bool LevelAcceleration(std::FILE* out, const AircraftType& type, const FlightDynamicsSystem::FuelCapacity& fuel,
                       const Band& band, double grossLbf) {
    // JSBSim's trim fails at the odd point the aircraft flies perfectly
    // well; a little faster usually trims, and the band still starts below
    // the reference data's slowest Mach.
    std::optional<testrig::AircraftRig> trimmed;
    for (double startMach = band.startMach; startMach < band.startMach + kTrimRetrySpanMach && !trimmed;
         startMach += kTrimRetryStepMach) {
        double kts = startMach * Atmosphere::Lookup(band.altitudeFt).speedOfSoundFtPerSec / Units::kFpsPerKt;
        try {
            trimmed.emplace(type.id.c_str(), band.altitudeFt, kts, 0.0);
        } catch (const std::exception&) {
            std::fprintf(stderr, "  no trim at %.0f ft, Mach %.2f\n", band.altitudeFt, startMach);
        }
    }
    if (!trimmed) return false;
    testrig::AircraftRig& rig = *trimmed;
    FlightDynamicsSystem::SetGrossWeight(rig.Fdm(), type, fuel, grossLbf);
    rig.Fdm().SetPropertyValue("propulsion/fuel_freeze", 1.0);
    rig.Fly(kSettleS);  // settle on the new weight
    rig.Autopilot().targetSpeedKts = kFullPowerCmdKts;

    double previousMach = 0.0;
    int ticks = static_cast<int>(1.0 / testrig::AircraftRig::kDt);
    for (int window = 0; window < kMaxWindows; ++window) {
        double drag = 0.0, alpha = 0.0, nz = 0.0, mach = 0.0, thrust = 0.0, altitude = 0.0;
        for (int i = 0; i < ticks; ++i) {
            rig.Step();
            drag += rig.Prop("forces/fwx-aero-lbs");
            alpha += rig.Prop("aero/alpha-deg");
            nz += rig.Prop("accelerations/Nz");
            mach += rig.Prop("velocities/mach");
            thrust += rig.Prop("propulsion/engine[0]/thrust-lbs");
            altitude += rig.Prop("position/h-sl-ft");
        }
        mach /= ticks;
        std::fprintf(out, "%.0f,%.0f,%.4f,%.1f,%.3f,%.4f,%.1f,%.1f,%.1f\n", band.altitudeFt, grossLbf, mach,
                     drag / ticks, alpha / ticks, nz / ticks, thrust / ticks, altitude / ticks,
                     rig.Prop("inertia/weight-lbs"));
        if (mach > band.maxMach) break;
        if (window % kPlateauWindows == kPlateauWindows - 1) {
            if (window > kPlateauWindows && mach - previousMach < kPlateauMach) break;
            previousMach = mach;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3 || std::strcmp(argv[1], "--aircraft") != 0) {
        std::fprintf(stderr, "usage: drag_sample --aircraft <type id under data/aircraft>\n");
        return 2;
    }
    JSBSim::FGJSBBase::debug_lvl = 0;
    try {
        std::shared_ptr<const AircraftType> type = AircraftCatalog::Default().Get(argv[2]);
        std::vector<Band> bands = BandsFromReference(type->directory / "reference" / "flight_manual_accel.csv");
        std::ifstream settingsFile(type->directory / "fit" / "thrust_fit.json");
        nlohmann::json settings = nlohmann::json::parse(settingsFile);
        std::vector<double> weights = settings.at("drag_sample_gross_weights_lb").get<std::vector<double>>();
        // Military thrust above the accelerations' altitudes is fitted to the
        // climb envelope when the settings say so; that needs drag there.
        if (settings.value("fit_mil_to_climb_envelope", false)) {
            for (const Band& band : BandsFromEnvelope(type->directory / "reference" / "climb_envelope.csv", "MIL")) {
                bands.push_back(band);
            }
        }
        FlightDynamicsSystem::FuelCapacity fuel = FlightDynamicsSystem::FuelCapacityOf(*type);

        std::filesystem::path outPath = type->directory / "fit" / "drag_1g.csv";
        std::FILE* out = std::fopen(outPath.string().c_str(), "w");
        if (out == nullptr) throw std::runtime_error("cannot write " + outPath.string());
        std::fprintf(out, "altitude_ft,gross_weight_lb,mach,drag_lbf,alpha_deg,nz,thrust_lbf,flown_altitude_ft,"
                          "weight_lb\n");
        for (const Band& band : bands) {
            int flown = 0;
            for (double gross : weights) {
                std::fprintf(stderr, "%.0f ft, %.0f lb\n", band.altitudeFt, gross);
                if (LevelAcceleration(out, *type, fuel, band, gross)) ++flown;
                std::fflush(out);
            }
            // The drag regression needs at least two weights per band.
            if (flown < 2) {
                throw std::runtime_error("fewer than two weights trim at " + std::to_string(band.altitudeFt) + " ft");
            }
        }
        std::fclose(out);
        std::fprintf(stderr, "wrote %s\n", outPath.string().c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "drag_sample: %s\n", e.what());
        return 1;
    }
    return 0;
}
