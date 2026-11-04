#pragma once

#include <memory>

#include "aircraft/AircraftType.h"

// Which aircraft type an entity is. Attached by FlightDynamicsSystem::Init
// to every aircraft it creates, so any entity with a JSBSimComponent has
// one. The type is shared and immutable: per-aircraft state (damage, fuel,
// the flight model's own state) lives in the other components.
struct AircraftComponent {
    std::shared_ptr<const AircraftType> type;
};
