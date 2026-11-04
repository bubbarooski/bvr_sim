#include "FlightDynamicsSystem.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "FGFDMExec.h"
#include "initialization/FGInitialCondition.h"
#include "initialization/FGTrim.h"
#include "models/FGMassBalance.h"
#include "models/FGPropagate.h"
#include "models/FGPropulsion.h"
#include "models/propulsion/FGTank.h"
#include "models/atmosphere/FGStandardAtmosphere.h"
#include "simgear/misc/sg_path.hxx"

#include "components/AircraftComponent.h"
#include "components/DestroyedComponent.h"
#include "components/JSBSimComponent.h"
#include "components/Transform.h"
#include "ecs/ComponentStore.h"
#include "sim/SimClock.h"

namespace {

// JSBSim integrates over lat/lon; the sim works in a local Cartesian play
// area. Bridged by a flat-Earth approximation referenced at (0,0), where a
// degree of latitude and longitude cover the same ground distance. See
// REFERENCE.md - Flat-Earth coordinate mapping.
constexpr double kRefLatDeg = 0.0;
constexpr double kRefLonDeg = 0.0;
constexpr double kFeetPerDegree = 365221.0;  // ~1 deg lat/lon at the equator

double LatDegFromNorthFt(double northFt) { return kRefLatDeg + northFt / kFeetPerDegree; }
double LonDegFromEastFt(double eastFt) { return kRefLonDeg + eastFt / kFeetPerDegree; }
double NorthFtFromLatDeg(double latDeg) { return (latDeg - kRefLatDeg) * kFeetPerDegree; }
double EastFtFromLonDeg(double lonDeg) { return (lonDeg - kRefLonDeg) * kFeetPerDegree; }

// One JSBSim step per physics tick.
constexpr double kJSBSimDt = SimClock::kFixedDt;

// Fills every tank whose name starts with the type's internal-tank prefix
// to the given fraction of its capacity; the rest (drop tanks) are left as
// the model file declares them. A model with no such tank cannot say which
// of its tanks are internal, so asking it to is an error rather than a
// guess.
void LoadInternalFuel(JSBSim::FGFDMExec& fdm, double fraction, const AircraftType& type,
                      const std::string& entityName) {
    // Zero is excluded: an aircraft spawned airborne on empty tanks has no
    // running engine and cannot even be trimmed.
    if (!(fraction > 0.0 && fraction <= 1.0)) {
        throw std::runtime_error("FlightDynamicsSystem: entity '" + entityName +
                                 "': internal fuel fraction must be above 0 and at most 1, got " +
                                 std::to_string(fraction));
    }
    auto propulsion = fdm.GetPropulsion();
    int filled = 0;
    for (unsigned int i = 0; i < propulsion->GetNumTanks(); ++i) {
        auto tank = propulsion->GetTank(i);
        if (tank->GetName().rfind(type.internalTankPrefix, 0) != 0) continue;
        tank->SetContents(tank->GetCapacity() * fraction);
        ++filled;
    }
    if (filled == 0) {
        throw std::runtime_error("FlightDynamicsSystem: entity '" + entityName +
                                 "' sets an internal fuel fraction, but aircraft type '" + type.id +
                                 "' has no tank named \"" + type.internalTankPrefix + " ...\"");
    }
}

}  // namespace

namespace FlightDynamicsSystem {

void SetTemperatureOffset(JSBSim::FGFDMExec& fdm, double temperatureOffsetC) {
    // Every model here gets JSBSim's default FGStandardAtmosphere, which is
    // where the bias lives; anything else would be a configuration error.
    auto atmosphere = std::dynamic_pointer_cast<JSBSim::FGStandardAtmosphere>(fdm.GetAtmosphere());
    if (atmosphere == nullptr) throw std::runtime_error("FlightDynamicsSystem: aircraft has no standard atmosphere");
    atmosphere->SetTemperatureBias(JSBSim::FGAtmosphere::eCelsius, temperatureOffsetC);
}

void SetWind(JSBSim::FGFDMExec& fdm, const Vec3& windFps) {
    // FGWinds takes the velocity of the air, north-east-down.
    fdm.SetPropertyValue("atmosphere/wind-north-fps", windFps.z());
    fdm.SetPropertyValue("atmosphere/wind-east-fps", windFps.x());
    fdm.SetPropertyValue("atmosphere/wind-down-fps", -windFps.y());
}

void ApplySpawnWind(JSBSim::FGFDMExec& fdm, const Vec3& windFps) {
    // A steady wind leaves the airflow over the airframe unchanged, so the
    // still-air trim is already the trim in wind: adding the wind to the
    // inertial velocity while FGWinds starts blowing keeps the air-relative
    // state exactly the trimmed one. The initial-condition route is
    // unusable: FGInitialCondition holds its wind as air-minus-ground
    // velocity, FGWinds as ground-minus-air, and Initialize (which trim
    // calls repeatedly) copies one into the other unconverted, so an aircraft
    // spawned that way flies the wrong airspeed at large sideslip and will
    // not trim. See REFERENCE.md - Wind.
    auto propagate = fdm.GetPropagate();
    JSBSim::FGColumnVector3 windNed(windFps.z(), windFps.x(), -windFps.y());
    propagate->SetInertialVelocity(propagate->GetInertialVelocity() + propagate->GetTl2i() * windNed);
    // The integrator's derivative history was seeded by trim from the
    // still-air velocity; reseed it from the shifted one.
    propagate->InitializeDerivatives();
    SetWind(fdm, windFps);
}

std::unique_ptr<JSBSim::FGFDMExec> CreateFlightModel(const AircraftType& type, const std::string& forWhat) {
    auto fdm = std::make_unique<JSBSim::FGFDMExec>();
    fdm->SetRootDir(SGPath("data/jsbsim_data"));

    // SetRootDir does NOT retroactively prefix the default
    // aircraft/engine/systems paths -- only these three setters run a path
    // through GetFullPath, which is what applies RootDir. Without them
    // LoadModel resolves relative to the process CWD.
    fdm->SetAircraftPath(SGPath("aircraft"));
    fdm->SetEnginePath(SGPath("engine"));
    fdm->SetSystemsPath(SGPath("systems"));

    fdm->Setdt(kJSBSimDt);

    const std::string& model = type.jsbsimModel;
    if (!fdm->LoadModel(model)) {
        throw std::runtime_error("FlightDynamicsSystem: failed to load JSBSim model '" + model + "' for " + forWhat +
                                  " -- expected data/jsbsim_data/aircraft/" + model + "/" + model + ".xml "
                                  "(plus its engine/thruster files under data/jsbsim_data/engine/)");
    }
    return fdm;
}

FuelCapacity FuelCapacityOf(const AircraftType& type) {
    std::unique_ptr<JSBSim::FGFDMExec> fdm = CreateFlightModel(type, "aircraft type '" + type.id + "'");
    FuelCapacity capacity;
    capacity.zeroFuelWeightLbf =
        fdm->GetMassBalance()->GetEmptyWeight() + fdm->GetMassBalance()->GetTotalPointMassWeight();
    auto propulsion = fdm->GetPropulsion();
    for (unsigned int i = 0; i < propulsion->GetNumTanks(); ++i) {
        auto tank = propulsion->GetTank(i);
        bool internal = tank->GetName().rfind(type.internalTankPrefix, 0) == 0;
        (internal ? capacity.internalLbf : capacity.externalLbf) += tank->GetCapacity();
    }
    return capacity;
}

void SetGrossWeight(JSBSim::FGFDMExec& fdm, const AircraftType& type, const FuelCapacity& capacity,
                    double grossLbf) {
    double fuel = grossLbf - capacity.zeroFuelWeightLbf;
    if (fuel <= 0.0 || fuel > capacity.internalLbf + capacity.externalLbf) {
        throw std::runtime_error("FlightDynamicsSystem: aircraft type '" + type.id + "' cannot weigh " +
                                 std::to_string(grossLbf) + " lb on fuel alone");
    }
    double internalFraction = std::min(fuel, capacity.internalLbf) / capacity.internalLbf;
    double externalFraction =
        capacity.externalLbf > 0.0 ? std::max(0.0, fuel - capacity.internalLbf) / capacity.externalLbf : 0.0;
    auto propulsion = fdm.GetPropulsion();
    for (unsigned int i = 0; i < propulsion->GetNumTanks(); ++i) {
        auto tank = propulsion->GetTank(i);
        bool internal = tank->GetName().rfind(type.internalTankPrefix, 0) == 0;
        tank->SetContents(tank->GetCapacity() * (internal ? internalFraction : externalFraction));
    }
}

void Init(World& world, EntityID entity, const ScenarioEntity& scenarioEntity) {
    if (scenarioEntity.jsbsim.type == nullptr) {
        throw std::runtime_error("FlightDynamicsSystem: entity '" + scenarioEntity.name + "' has no aircraft type");
    }
    const AircraftType& type = *scenarioEntity.jsbsim.type;
    const std::string& model = type.jsbsimModel;
    std::unique_ptr<JSBSim::FGFDMExec> fdm = CreateFlightModel(type, "entity '" + scenarioEntity.name + "'");

    auto ic = fdm->GetIC();
    ic->SetLatitudeDegIC(LatDegFromNorthFt(scenarioEntity.pos.z()));
    ic->SetLongitudeDegIC(LonDegFromEastFt(scenarioEntity.pos.x()));
    ic->SetAltitudeASLFtIC(scenarioEntity.jsbsim.altitudeFt);
    ic->SetPsiDegIC(scenarioEntity.jsbsim.headingDeg);
    ic->SetThetaDegIC(0.0);
    ic->SetPhiDegIC(0.0);
    ic->SetVtrueKtsIC(scenarioEntity.jsbsim.airspeedKts);

    // Without this the engine starts cold and idle while the airframe is
    // already at cruise speed, which is enough to stall it outright before
    // the autopilot can matter.
    constexpr int kAllEngines = -1;
    fdm->SetPropertyValue("propulsion/set-running", kAllEngines);

    // JSBSim spawns every model gear-down, and gear drag (the F-16's CDgear
    // term adds ~60% to clean parasite drag) then lasts the whole flight.
    // Every entity spawns airborne, so retract it before trim solves around it.
    fdm->SetPropertyValue("gear/gear-cmd-norm", 0.0);

    // The scenario's air, before trim, so trim solves for the day it will
    // actually fly in rather than a standard one.
    SetTemperatureOffset(*fdm, scenarioEntity.jsbsim.temperatureOffsetC);

    // Fuel before trim too, so trim solves at the weight the aircraft flies.
    if (scenarioEntity.jsbsim.internalFuelFraction) {
        LoadInternalFuel(*fdm, *scenarioEntity.jsbsim.internalFuelFraction, type, scenarioEntity.name);
    }

    if (!fdm->RunIC()) {
        throw std::runtime_error("FlightDynamicsSystem: RunIC() failed for entity '" + scenarioEntity.name +
                                  "' (model '" + model + "')");
    }

    // RunIC sets kinematic state only -- it does not solve for the
    // elevator/throttle/alpha that balance the aircraft there. Without
    // trim the autopilot inherits a large initial force imbalance to
    // recover from rather than small deviations to correct.
    try {
        fdm->DoTrim(JSBSim::tFull);
    } catch (const std::exception& e) {
        throw std::runtime_error("FlightDynamicsSystem: trim failed for entity '" + scenarioEntity.name +
                                  "' (model '" + model + "'): " + e.what());
    }

    JSBSimComponent component;
    // Resolve every hot-path property to its node once, rather than
    // re-parsing the path string on each per-tick access. create=true so a
    // missing property reads 0.0 instead of null-dereferencing.
    JSBSim::FGPropertyManager* pm = fdm->GetPropertyManager().get();
    component.props.latGcDeg = pm->GetNode("position/lat-gc-deg", true);
    component.props.longGcDeg = pm->GetNode("position/long-gc-deg", true);
    component.props.hSlFt = pm->GetNode("position/h-sl-ft", true);
    component.props.phiDeg = pm->GetNode("attitude/phi-deg", true);
    component.props.thetaDeg = pm->GetNode("attitude/theta-deg", true);
    component.props.psiDeg = pm->GetNode("attitude/psi-deg", true);
    component.props.pRadSec = pm->GetNode("velocities/p-rad_sec", true);
    component.props.qRadSec = pm->GetNode("velocities/q-rad_sec", true);
    component.props.vtKts = pm->GetNode("velocities/vtrue-kts", true);
    component.props.vtFps = pm->GetNode("velocities/vt-fps", true);
    component.props.vNorthFps = pm->GetNode("velocities/v-north-fps", true);
    component.props.vEastFps = pm->GetNode("velocities/v-east-fps", true);
    component.props.vDownFps = pm->GetNode("velocities/v-down-fps", true);
    component.props.nz = pm->GetNode("accelerations/Nz", true);
    component.props.nx = pm->GetNode("accelerations/Nx", true);
    component.props.ny = pm->GetNode("accelerations/Ny", true);
    component.props.weightLbs = pm->GetNode("inertia/weight-lbs", true);
    component.props.aileronCmd = pm->GetNode("fcs/aileron-cmd-norm", true);
    component.props.elevatorCmd = pm->GetNode("fcs/elevator-cmd-norm", true);
    component.props.throttleCmd = pm->GetNode("fcs/throttle-cmd-norm[0]", true);
    // create=false: absent stays null rather than a property reading 0,
    // which would zero the envelope's lift.
    component.props.clMaxMachFactor = pm->GetNode("aero/function/kCLmax-mach", false);

    // The trim throttle becomes ControlSystem's feed-forward baseline.
    // Read after DoTrim and before any Run, so this is the equilibrium
    // itself rather than a later disturbed state.
    component.trimThrottle = component.props.throttleCmd->getDoubleValue();
    component.appliedTemperatureOffsetC = scenarioEntity.jsbsim.temperatureOffsetC;

    component.fdm = std::move(fdm);
    world.Components<JSBSimComponent>().Add(entity, std::move(component));
    world.Components<AircraftComponent>().Add(entity, AircraftComponent{scenarioEntity.jsbsim.type});
}

void Update(World& world, double dt) {
    // dt == 0 (paused) must mean zero steps; the max(1, ...) below only
    // keeps a small positive dt from rounding to none.
    if (dt <= 0.0) return;

    ComponentStore<JSBSimComponent>& jsbsimComponents = world.Components<JSBSimComponent>();
    ComponentStore<Transform>& transforms = world.Components<Transform>();
    ComponentStore<DestroyedComponent>& destroyed = world.Components<DestroyedComponent>();

    // Round, not truncate: a dt a hair off kJSBSimDt would otherwise land
    // on a lower step count and silently run slow.
    int subSteps = std::max(1, static_cast<int>(std::lround(dt / kJSBSimDt)));

    for (auto& [entity, component] : jsbsimComponents) {
        // A destroyed aircraft freezes where it was hit.
        if (destroyed.Has(entity)) continue;
        JSBSim::FGFDMExec* fdm = component.fdm.get();
        for (int i = 0; i < subSteps; ++i) {
            fdm->Run();
        }

        Transform* transformPtr = transforms.TryGet(entity);
        if (transformPtr == nullptr) continue;
        Transform& transform = *transformPtr;

        double latDeg = component.props.latGcDeg->getDoubleValue();
        double lonDeg = component.props.longGcDeg->getDoubleValue();
        double altFt = component.props.hSlFt->getDoubleValue();
        double phiDeg = component.props.phiDeg->getDoubleValue();
        double thetaDeg = component.props.thetaDeg->getDoubleValue();
        double psiDeg = component.props.psiDeg->getDoubleValue();

        transform.pos = Vec3{EastFtFromLonDeg(lonDeg), altFt, NorthFtFromLatDeg(latDeg)};
        transform.euler = Vec3{phiDeg, thetaDeg, psiDeg};
    }
}

}  // namespace FlightDynamicsSystem
