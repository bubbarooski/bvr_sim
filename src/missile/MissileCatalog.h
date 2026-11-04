#pragma once

#include <filesystem>

#include "missile/MissileType.h"
#include "util/TypeCatalog.h"

// The missile types available to scenarios: data/missiles/<id>/missile.json
// for each, loaded once and shared (see TypeCatalog.h).
class MissileCatalog : public TypeCatalog<MissileType> {
public:
    explicit MissileCatalog(std::filesystem::path root);

    // The catalog everything uses unless handed another: data/missiles.
    static MissileCatalog& Default();
};
