#include "EnvironmentSystem.h"

#include "components/JSBSimComponent.h"
#include "ecs/ComponentStore.h"
#include "systems/FlightDynamicsSystem.h"

namespace EnvironmentSystem {

void Update(World& world, SimContext& simContext) {
    const Environment& environment = simContext.environment;
    const Vec3 windFps = WindFps(environment);
    for (auto& [entity, jsbsim] : world.Components<JSBSimComponent>()) {
        if (jsbsim.appliedTemperatureOffsetC != environment.temperatureOffsetC) {
            FlightDynamicsSystem::SetTemperatureOffset(*jsbsim.fdm, environment.temperatureOffsetC);
            jsbsim.appliedTemperatureOffsetC = environment.temperatureOffsetC;
        }

        const Vec3& applied = jsbsim.appliedWindFps;
        bool changed = applied.x() != windFps.x() || applied.y() != windFps.y() || applied.z() != windFps.z();
        if (!jsbsim.spawnWindApplied) {
            if (changed) FlightDynamicsSystem::ApplySpawnWind(*jsbsim.fdm, windFps);
            jsbsim.appliedWindFps = windFps;
            jsbsim.spawnWindApplied = true;
        } else if (changed) {
            FlightDynamicsSystem::SetWind(*jsbsim.fdm, windFps);
            jsbsim.appliedWindFps = windFps;
        }
    }
}

}  // namespace EnvironmentSystem
