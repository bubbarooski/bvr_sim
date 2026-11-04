#pragma once

#include <filesystem>

#include "aircraft/AircraftType.h"
#include "util/TypeCatalog.h"

// The aircraft types available to scenarios: data/aircraft/<id>/aircraft.json
// for each, loaded once and shared (see TypeCatalog.h).
class AircraftCatalog : public TypeCatalog<AircraftType> {
public:
    explicit AircraftCatalog(std::filesystem::path root);

    // The catalog everything uses unless handed another: data/aircraft.
    static AircraftCatalog& Default();
};
