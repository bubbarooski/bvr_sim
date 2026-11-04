// Bare-airframe short-period identification for an aircraft type's pitch
// FCS design (the load-factor law in the F-16's model).
//
// For each (altitude, Mach) on a grid: trim the aircraft, bypass the pitch
// FCS (fcs/fbw-override), fly a small 3-2-1-1 elevator input, and fit the
// linear short-period model by least squares:
//
//   qdot     = Ma*da + Mq*q + Md*dd
//   alphadot = Za*da + Zq*q + Zd*dd
//   dn       = Na*da + Nq*q + Nd*dd
//
// where da is alpha - trim alpha (rad), q pitch rate (rad/s), dd elevator
// position - trim (normalized, +1 = 25 deg trailing-edge down) and dn load
// factor - trim load factor (G). The relaxed-stability airframe is held
// near trim during the input by a weak alpha + pitch-rate stabiliser of
// its own; least squares on the ACTUAL surface position still identifies
// the open-loop plant. tools/fit/fcs_design.py turns the CSV into gains.
//
// Writes data/aircraft/<id>/fit/fcs_plant.csv. Run from the repo root
// (data paths are relative to it):
//   build/Release/tools/fcs_identify.exe --aircraft f16c_block50

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "initialization/FGTrim.h"

#include "aircraft/AircraftCatalog.h"
#include "sim/SimClock.h"
#include "systems/FlightDynamicsSystem.h"
#include "util/Units.h"

namespace {

constexpr double kDt = SimClock::kFixedDt;
constexpr double kInputStartS = 0.5;
constexpr double kInputStepS = 0.2;  // 3-2-1-1 unit; spans the 2-8 rad/s short period
constexpr double kInputAmplitude = 0.02;  // normalized elevator, ~0.5 deg
constexpr double kRecordEndS = 4.0;

// The weak stabiliser: gains kAlphaGain, kQGain (elevator per rad, per
// rad/s) scaled by min(kMaxGainScale, kGainRefQbarPsf / max(qbar,
// kMinQbarPsf)), so its loop gain is similar at every dynamic pressure.
constexpr double kAlphaGain = 0.6;
constexpr double kQGain = 0.25;
constexpr double kMaxGainScale = 3.0;
constexpr double kGainRefQbarPsf = 300.0;
constexpr double kMinQbarPsf = 30.0;
// Wings held level: aileron per rad of bank and per rad/s of roll rate.
constexpr double kBankGain = 0.5;
constexpr double kRollRateGain = 0.2;

constexpr int kFitColumns = 17;  // the CSV columns after "trimmed"

// 3-2-1-1: +3 units, -2, +1, -1. Rich in frequency, zero mean.
double Doublet3211(double t) {
    double u = (t - kInputStartS) / kInputStepS;
    if (u < 0.0) return 0.0;
    if (u < 3.0) return kInputAmplitude;
    if (u < 5.0) return -kInputAmplitude;
    if (u < 6.0) return kInputAmplitude;
    if (u < 7.0) return -kInputAmplitude;
    return 0.0;
}

struct Sample {
    double da, q, dd, alphaDot, qDot, dn;
};

struct Fit {
    double c[3] = {0.0, 0.0, 0.0};  // on (da, q, dd)
    double r2 = 0.0;
};

// Least squares with a free bias column, so a slowly drifting trim does
// not leak into the derivatives.
Fit FitColumn(const std::vector<Sample>& samples, double Sample::*target) {
    const int n = static_cast<int>(samples.size());
    Eigen::MatrixXd x(n, 4);
    Eigen::VectorXd y(n);
    for (int i = 0; i < n; ++i) {
        x(i, 0) = samples[i].da;
        x(i, 1) = samples[i].q;
        x(i, 2) = samples[i].dd;
        x(i, 3) = 1.0;
        y(i) = samples[i].*target;
    }
    Eigen::VectorXd beta = x.colPivHouseholderQr().solve(y);
    Eigen::VectorXd residual = y - x * beta;
    double variance = (y.array() - y.mean()).square().sum();
    Fit fit;
    for (int k = 0; k < 3; ++k) fit.c[k] = beta(k);
    fit.r2 = variance > 0.0 ? 1.0 - residual.squaredNorm() / variance : 0.0;
    return fit;
}

// The type's flight model trimmed level at one grid point, or nullptr if it
// will not trim there (beyond the envelope).
std::unique_ptr<JSBSim::FGFDMExec> TrimmedAircraft(const AircraftType& type, double altitudeFt, double mach) {
    std::unique_ptr<JSBSim::FGFDMExec> fdm = FlightDynamicsSystem::CreateFlightModel(type, "fcs_identify");
    fdm->SetDebugLevel(0);
    auto ic = fdm->GetIC();
    ic->SetAltitudeASLFtIC(altitudeFt);
    ic->SetMachIC(mach);
    ic->SetThetaDegIC(0.0);
    ic->SetPhiDegIC(0.0);
    fdm->SetPropertyValue("propulsion/set-running", -1);
    fdm->SetPropertyValue("gear/gear-cmd-norm", 0.0);
    // Trimmed with the pitch FCS already bypassed, so trim sets the surface
    // directly and finds the bare airframe's own equilibrium. Through the
    // FCS the solution would depend on the model's gains (two solutions at
    // a few transonic points), and identify/design would never reach a
    // fixed point.
    fdm->SetPropertyValue("fcs/fbw-override", 1.0);
    if (!fdm->RunIC()) return nullptr;
    try {
        fdm->DoTrim(JSBSim::tFull);
    } catch (const std::exception&) {
        return nullptr;
    }
    return fdm;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3 || std::string(argv[1]) != "--aircraft") {
        std::fprintf(stderr, "usage: fcs_identify --aircraft <type id under data/aircraft>\n");
        return 2;
    }
    // JSBSim prints a banner per instance; keep the console readable.
#ifdef _WIN32
    _putenv("JSBSIM_DEBUG=0");
#else
    setenv("JSBSIM_DEBUG", "0", 1);
#endif
    JSBSim::FGJSBBase::debug_lvl = 0;
    std::shared_ptr<const AircraftType> type;
    try {
        type = AircraftCatalog::Default().Get(argv[2]);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fcs_identify: %s\n", e.what());
        return 1;
    }
    std::string outPath = (type->directory / "fit" / "fcs_plant.csv").string();
    std::FILE* out = std::fopen(outPath.c_str(), "w");
    if (out == nullptr) {
        std::fprintf(stderr, "fcs_identify: cannot write %s\n", outPath.c_str());
        return 1;
    }

    const double altitudesFt[] = {1000.0, 10000.0, 20000.0, 30000.0, 40000.0, 50000.0};
    const double machs[] = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.85, 0.9, 0.95, 1.0,
                            1.05, 1.1, 1.2, 1.4, 1.6, 1.8, 2.0};

    std::fprintf(out, "alt_ft,mach,trimmed,qbar_psf,vt_fps,alpha0_deg,elevator0,"
                "Ma,Mq,Md,Za,Zq,Zd,Na,Nq,Nd,r2_qdot,r2_alphadot,r2_n,max_dalpha_deg\n");
    for (double altitudeFt : altitudesFt) {
        for (double mach : machs) {
            auto fdm = TrimmedAircraft(*type, altitudeFt, mach);
            if (fdm == nullptr) {
                std::fprintf(out, "%.0f,%.2f,0%s\n", altitudeFt, mach, std::string(kFitColumns, ',').c_str());
                continue;
            }
            auto prop = [&](const char* name) { return fdm->GetPropertyValue(name); };
            const double alpha0 = prop("aero/alpha-rad");
            const double elevator0 = prop("fcs/elevator-pos-norm");
            const double n0 = prop("accelerations/Nz");
            const double qbar = prop("aero/qbar-psf");
            const double vt = prop("velocities/vt-fps");

            // Positive elevator is nose-down, so alpha or pitch rate above
            // trim takes positive elevator.
            const double scale = std::min(kMaxGainScale, kGainRefQbarPsf / std::max(qbar, kMinQbarPsf));
            const double kAlpha = kAlphaGain * scale, kQ = kQGain * scale;

            fdm->SetPropertyValue("fcs/fbw-override", 1.0);
            std::vector<Sample> samples;
            double maxDalphaDeg = 0.0;
            for (double t = 0.0; t < kRecordEndS; t += kDt) {
                double da = prop("aero/alpha-rad") - alpha0;
                double q = prop("velocities/q-aero-rad_sec");
                fdm->SetPropertyValue("fcs/elevator-cmd-norm", kAlpha * da + kQ * q + Doublet3211(t));
                // Wings held level: the roll channel shares the override.
                fdm->SetPropertyValue("fcs/aileron-cmd-norm",
                                      -kBankGain * prop("attitude/phi-rad") -
                                          kRollRateGain * prop("velocities/p-aero-rad_sec"));
                fdm->Run();
                if (t >= kInputStartS) {
                    samples.push_back({prop("aero/alpha-rad") - alpha0, prop("velocities/q-aero-rad_sec"),
                                       prop("fcs/elevator-pos-norm") - elevator0, prop("aero/alphadot-rad_sec"),
                                       prop("accelerations/qdot-rad_sec2"), prop("accelerations/Nz") - n0});
                    maxDalphaDeg = std::max(maxDalphaDeg, std::fabs(samples.back().da) * Units::kRadToDeg);
                }
            }
            Fit m = FitColumn(samples, &Sample::qDot);
            Fit z = FitColumn(samples, &Sample::alphaDot);
            Fit nz = FitColumn(samples, &Sample::dn);
            std::fprintf(out, "%.0f,%.2f,1,%.1f,%.1f,%.3f,%.5f,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5g,%.5f,%.5f,%.5f,%.3f\n",
                        altitudeFt, mach, qbar, vt, alpha0 * Units::kRadToDeg, elevator0, m.c[0], m.c[1], m.c[2], z.c[0],
                        z.c[1], z.c[2], nz.c[0], nz.c[1], nz.c[2], m.r2, z.r2, nz.r2, maxDalphaDeg);
            std::fprintf(stderr, "  %5.0f ft  M%.2f\n", altitudeFt, mach);
        }
    }
    std::fclose(out);
    return 0;
}
