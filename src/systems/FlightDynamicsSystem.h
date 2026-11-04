#pragma once

#include <memory>
#include <string>

#include "aircraft/AircraftType.h"
#include "ecs/World.h"
#include "scenario/ScenarioEntity.h"

namespace JSBSim {
class FGFDMExec;
}

// Owns the JSBSim side of aircraft entities: instantiates and configures
// one FGFDMExec per entity at load time, then steps it every tick and
// writes the result back into Transform.
namespace FlightDynamicsSystem {

// A type's JSBSim model, loaded and configured (data paths, step size) but
// not initialised or trimmed. Throws, naming `forWhat` (an entity, a tool's
// purpose), if the model cannot be loaded. Init builds on this; the
// model-fitting tools use it directly.
std::unique_ptr<JSBSim::FGFDMExec> CreateFlightModel(const AircraftType& type, const std::string& forWhat);

// What the type's flight model weighs without fuel, and how much fuel its
// tanks hold, split into internal tanks (the type's prefix) and the rest.
// Read from the model file, so nothing about the airframe's weights is
// repeated anywhere else.
struct FuelCapacity {
    double zeroFuelWeightLbf = 0.0;  // empty weight plus fixed point masses (pilot)
    double internalLbf = 0.0;
    double externalLbf = 0.0;
};
FuelCapacity FuelCapacityOf(const AircraftType& type);

// Loads fuel so the aircraft weighs `grossLbf`: internal tanks first, all
// to the same fraction, then external tanks equally. Throws if fuel alone
// cannot reach that weight. For test rigs and fitting tools matching a
// published gross weight; scenarios fly the type's own load.
void SetGrossWeight(JSBSim::FGFDMExec& fdm, const AircraftType& type, const FuelCapacity& capacity, double grossLbf);

// Creates an FGFDMExec for `entity` from `scenarioEntity.jsbsim` -- its
// aircraft type's flight model -- sets its initial condition, trims it, and
// attaches the resulting JSBSimComponent plus an AircraftComponent naming
// the type.
// Throws std::runtime_error naming the specific failure -- a bad model or
// an unsolvable trim must fail loudly, not produce a dead aircraft.
void Init(World& world, EntityID entity, const ScenarioEntity& scenarioEntity);

// Applies an ISA temperature offset to one aircraft's own JSBSim
// atmosphere -- the single place that knows how JSBSim takes it.
void SetTemperatureOffset(JSBSim::FGFDMExec& fdm, double temperatureOffsetC);

// Sets the wind (world frame: x east, y up, z north; ft/s) one aircraft's
// JSBSim is flying through. A change mid-flight is the air changing around
// the aircraft, so its ground velocity is left alone.
void SetWind(JSBSim::FGFDMExec& fdm, const Vec3& windFps);

// Puts a freshly trimmed aircraft into its spawn wind: sets the wind and
// moves its ground velocity with it, so it starts at its trimmed airspeed
// through the moving air rather than being hit by a gust at spawn.
void ApplySpawnWind(JSBSim::FGFDMExec& fdm, const Vec3& windFps);

// Steps every entity's FGFDMExec enough JSBSim sub-steps to cover dt, then
// writes position/attitude back into that entity's Transform.
void Update(World& world, double dt);

}  // namespace FlightDynamicsSystem
