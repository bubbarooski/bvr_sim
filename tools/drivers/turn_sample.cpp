// Drag at a flight manual's sustained-turn load factors, for checking and
// fitting an aircraft type's drag due to lift.
//
// A sustained turn is where full-power thrust just equals drag at that load
// factor. Thrust is already fitted at 1 g (tools/fit/thrust_fit.py), so the
// manual's sustained turn says what the drag coefficient is at that lift
// coefficient: CD = T cos(alpha) / (q S). For each point in
// reference/turn_ps0.csv this flies the type, frozen at the point's gross
// weight, at the point's Mach: full-power thrust at 1 g, then a quick pull
// to 0.85, 1.0 and 1.15 x the manual's load factor, recording the drag and
// lift coefficients and alpha there.
//
// Output: data/aircraft/<id>/fit/turn_drag.csv. tools/fit/lift_drag_fit.py fits
// the drag correction from it. Run from the repo root:
//   build/Release/tools/turn_sample.exe --aircraft f16c_block50

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "FGFDMExec.h"

#include "aircraft/AircraftCatalog.h"
#include "sim/Atmosphere.h"
#include "systems/FlightDynamicsSystem.h"
#include "util/Units.h"
#include "validation/AircraftRig.h"
#include "validation/TurnDragSampler.h"

namespace {

// Each manual point is flown at these multiples of its load factor.
constexpr double kLoadFactorScales[] = {0.85, 1.0, 1.15};

struct ManualPoint {
    double altitudeFt, mach, grossLbf, rateDegPerSec;
};

std::vector<ManualPoint> ReadPoints(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    std::string line;
    std::getline(in, line);  // header: altitude_ft,mach,gross_weight_lb,rate_deg_s,...
    std::vector<ManualPoint> points;
    while (std::getline(in, line)) {
        std::stringstream row(line);
        std::string cell;
        std::vector<double> v;
        for (int i = 0; i < 4 && std::getline(row, cell, ','); ++i) v.push_back(std::stod(cell));
        points.push_back({v[0], v[1], v[2], v[3]});
    }
    return points;
}

// Load factor of a level turn at this rate and true airspeed.
double LoadFactor(double rateDegPerSec, double speedFtPerSec) {
    double omegaVOverG = rateDegPerSec * Units::kDegToRad * speedFtPerSec / Units::kGravityFtPerSec2;
    return std::sqrt(omegaVOverG * omegaVOverG + 1.0);
}

void FlyPoint(std::FILE* out, const AircraftType& type, const FlightDynamicsSystem::FuelCapacity& fuel,
              const ManualPoint& point, double loadFactor) {
    std::optional<testrig::TurnDragSample> sample =
        testrig::SampleTurnDrag(type, fuel, point.altitudeFt, point.mach, point.grossLbf, loadFactor);
    if (!sample) {
        std::fprintf(stderr, "  no trim at %.0f ft, Mach %.2f\n", point.altitudeFt, point.mach);
        return;
    }
    std::fprintf(out, "%.0f,%.2f,%.0f,%.3f,%.4f,%.1f,%.5f,%.4f,%.3f,%.3f,%d\n", point.altitudeFt, point.mach,
                 point.grossLbf, loadFactor, sample->mach, sample->thrustLbf, sample->cd, sample->cl,
                 sample->alphaDeg, sample->nz, sample->stickAtStop ? 1 : 0);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3 || std::strcmp(argv[1], "--aircraft") != 0) {
        std::fprintf(stderr, "usage: turn_sample --aircraft <type id under data/aircraft>\n");
        return 2;
    }
    JSBSim::FGJSBBase::debug_lvl = 0;
    try {
        std::shared_ptr<const AircraftType> type = AircraftCatalog::Default().Get(argv[2]);
        std::vector<ManualPoint> points = ReadPoints(type->directory / "reference" / "turn_ps0.csv");
        FlightDynamicsSystem::FuelCapacity fuel = FlightDynamicsSystem::FuelCapacityOf(*type);
        std::filesystem::path outPath = type->directory / "fit" / "turn_drag.csv";
        std::FILE* out = std::fopen(outPath.string().c_str(), "w");
        if (out == nullptr) throw std::runtime_error("cannot write " + outPath.string());
        std::fprintf(out, "altitude_ft,point_mach,gross_weight_lb,load_factor_cmd,mach,thrust_lbf,"
                          "drag_coefficient,lift_coefficient,alpha_deg,nz,stick_at_stop\n");
        for (const ManualPoint& point : points) {
            double speed = point.mach * Atmosphere::Lookup(point.altitudeFt).speedOfSoundFtPerSec;
            double manualN = LoadFactor(point.rateDegPerSec, speed);
            std::fprintf(stderr, "%.0f ft M%.2f: manual %.2f G\n", point.altitudeFt, point.mach, manualN);
            for (double scale : kLoadFactorScales) FlyPoint(out, *type, fuel, point, manualN * scale);
            std::fflush(out);
        }
        std::fclose(out);
        std::fprintf(stderr, "wrote %s\n", outPath.string().c_str());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "turn_sample: %s\n", e.what());
        return 1;
    }
    return 0;
}
