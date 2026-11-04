// Steady-cruise sampler for fitting an aircraft type's part-throttle fuel
// flow.
//
// A flight manual's cruise tables give total fuel flow in steady level
// flight, where thrust equals drag, so they measure fuel flow at part
// throttle. This tool flies each tabulated point (altitude, true airspeed,
// gross weight) in the model, the autopilot holding altitude and speed with
// fuel frozen at the gross weight, and records the thrust and engine speed
// the model needs there. tools/fit/fuel_fit.py fits the part-throttle factor
// with it. Sea level is flown at 200 ft, clear of the terrain.
//
// Inputs, under data/aircraft/<id>/:
//   reference/cruise_fuel.csv  altitude_ft, gross_weight_lb, ktas, fuel_flow_pph
// Output: fit/cruise_thrust.csv.
//
// Run from the repo root (data paths are relative to it):
//   build/Release/tools/cruise_sample.exe --aircraft f16c_block50

#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "FGFDMExec.h"

#include "aircraft/AircraftCatalog.h"
#include "systems/FlightDynamicsSystem.h"
#include "util/Units.h"
#include "validation/AircraftRig.h"

namespace {

constexpr double kSeaLevelFlownFt = 200.0;
constexpr double kSettleS = 40.0;  // the new weight, and JSBSim's slew of fuel flow
constexpr double kAverageS = 20.0;
constexpr double kSecondsPerHour = 3600.0;
// The autopilot held the tabulated speed to within this fraction.
constexpr double kSteadySpeedFraction = 0.02;

// Flies one point; false if it does not trim or the speed does not hold.
bool Cruise(std::FILE* out, const AircraftType& type, const FlightDynamicsSystem::FuelCapacity& fuel, double altitudeFt,
            double grossLbf, double ktas) {
    double flown = altitudeFt < kSeaLevelFlownFt ? kSeaLevelFlownFt : altitudeFt;
    try {
        testrig::AircraftRig rig(type.id.c_str(), flown, ktas, 0.0);
        FlightDynamicsSystem::SetGrossWeight(rig.Fdm(), type, fuel, grossLbf);
        rig.Fly(kSettleS);
        const int samples = static_cast<int>(kAverageS / testrig::AircraftRig::kDt);
        double thrust = 0.0, n2 = 0.0, mach = 0.0, vt = 0.0, flow = 0.0;
        for (int i = 0; i < samples; ++i) {
            rig.Step();
            thrust += rig.Prop("propulsion/engine[0]/thrust-lbs");
            n2 += rig.Prop("propulsion/engine[0]/n2");
            mach += rig.Prop("velocities/mach");
            vt += rig.Prop("velocities/vt-fps") / Units::kFpsPerKt;
            flow += rig.Prop("propulsion/engine[0]/fuel-flow-rate-pps") * kSecondsPerHour;
        }
        thrust /= samples; n2 /= samples; mach /= samples; vt /= samples; flow /= samples;
        bool steady = std::fabs(vt - ktas) < kSteadySpeedFraction * ktas;
        std::fprintf(out, "%.0f,%.0f,%.0f,%.4f,%.1f,%.2f,%.1f,%d\n", altitudeFt, grossLbf, ktas, mach, thrust, n2, flow,
                     steady ? 1 : 0);
        return steady;
    } catch (const std::exception&) {
        return false;
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3 || std::strcmp(argv[1], "--aircraft") != 0) {
        std::fprintf(stderr, "usage: cruise_sample --aircraft <type id under data/aircraft>\n");
        return 2;
    }
    JSBSim::FGJSBBase::debug_lvl = 0;
    try {
        std::shared_ptr<const AircraftType> type = AircraftCatalog::Default().Get(argv[2]);
        std::ifstream in(type->directory / "reference" / "cruise_fuel.csv");
        if (!in) throw std::runtime_error("no reference/cruise_fuel.csv");
        FlightDynamicsSystem::FuelCapacity fuel = FlightDynamicsSystem::FuelCapacityOf(*type);
        std::filesystem::path outPath = type->directory / "fit" / "cruise_thrust.csv";
        std::FILE* out = std::fopen(outPath.string().c_str(), "w");
        if (out == nullptr) throw std::runtime_error("cannot write " + outPath.string());
        std::fprintf(out, "altitude_ft,gross_weight_lb,ktas,mach,thrust_lbf,n2,flow_pph,steady\n");
        std::string line;
        std::getline(in, line);  // header
        int flown = 0, skipped = 0;
        while (std::getline(in, line)) {
            std::istringstream fields(line);
            std::string altitude, gross, ktas;
            if (!std::getline(fields, altitude, ',') || !std::getline(fields, gross, ',') ||
                !std::getline(fields, ktas, ',')) {
                continue;
            }
            if (Cruise(out, *type, fuel, std::stod(altitude), std::stod(gross), std::stod(ktas))) {
                ++flown;
            } else {
                ++skipped;
            }
            std::fflush(out);
        }
        std::fclose(out);
        std::fprintf(stderr, "wrote %s: %d steady points, %d not steady or not trimmed\n", outPath.string().c_str(), flown,
                     skipped);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "cruise_sample: %s\n", e.what());
        return 1;
    }
    return 0;
}
