#include <cmath>

#include <gtest/gtest.h>

#include "ecs/World.h"
#include "missile/MissileCatalog.h"
#include "scenario/ParamRegistry.h"
#include "scenario/ScenarioLoader.h"

TEST(ResolvePointer, DoctrineEntryIsUnchanged) {
    ParamRegistry::ParamDef param;
    param.pointer = "/missile_doctrine/kill_radius_m";
    EXPECT_EQ(ParamRegistry::ResolvePointer(param, -1), "/missile_doctrine/kill_radius_m");
}

TEST(ResolvePointer, SubstitutesEntityIndex) {
    ParamRegistry::ParamDef param;
    param.pointer = "/entities/{}/components/g_limit/structural_g_limit_pos";
    EXPECT_EQ(ParamRegistry::ResolvePointer(param, 3),
              "/entities/3/components/g_limit/structural_g_limit_pos");
}

TEST(ResolvePointer, IndexZeroStillSubstitutes) {
    ParamRegistry::ParamDef param;
    param.pointer = "/entities/{}/components/airborne_radar/search_threshold_w";
    EXPECT_EQ(ParamRegistry::ResolvePointer(param, 0), "/entities/0/components/airborne_radar/search_threshold_w");
}

TEST(ResolvePointer, OnlyFirstPlaceholderIsReplaced) {
    ParamRegistry::ParamDef param;
    param.pointer = "/{}/nested/{}";
    EXPECT_EQ(ParamRegistry::ResolvePointer(param, 7), "/7/nested/{}");
}

TEST(ParamRegistryTable, AllIsNonEmptyWithNoNullPointers) {
    const std::vector<ParamRegistry::ParamDef>& all = ParamRegistry::All();
    EXPECT_FALSE(all.empty());
    for (const ParamRegistry::ParamDef& param : all) {
        EXPECT_NE(param.pointer, nullptr);
    }
}

// The teaching knobs (docs/SCHEMA.md - Teaching knobs): 17 of them, plus
// the four placement fields. Everything else is a read-only type value.
TEST(ParamRegistryTable, OnlyTheTeachingKnobsAreEditable) {
    int knobs = 0;
    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        if (param.knob) ++knobs;
    }
    EXPECT_EQ(knobs, 21);

    const ParamRegistry::ParamDef* gain = ParamRegistry::Find("/missile_doctrine/nav_gain");
    ASSERT_NE(gain, nullptr);
    EXPECT_TRUE(gain->knob);
    const ParamRegistry::ParamDef* loss = ParamRegistry::Find("/entities/2/components/airborne_radar/system_loss_db");
    ASSERT_NE(loss, nullptr);
    EXPECT_FALSE(loss->knob);
    EXPECT_EQ(ParamRegistry::Find("/entities/x/components/airborne_radar/system_loss_db"), nullptr);
}

// Six knobs are Advanced-mode only in the GUI; only a knob can be one.
TEST(ParamRegistryTable, AdvancedKnobsAreKnobs) {
    int advanced = 0;
    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        if (!param.advanced) continue;
        ++advanced;
        EXPECT_TRUE(param.knob) << param.pointer;
    }
    EXPECT_EQ(advanced, 6);
    EXPECT_FALSE(ParamRegistry::Find("/missile_doctrine/nav_gain")->advanced);
    EXPECT_TRUE(ParamRegistry::Find("/missile_doctrine/kill_radius_m")->advanced);
}

// The loader applies an entity block's keys as Doubles only; an entity
// Bool or Enum entry would load as its default without a word.
TEST(ParamRegistryTable, EntityParamsAreDoubles) {
    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        if (param.owner == ParamRegistry::ParamOwner::Entity) {
            EXPECT_EQ(param.type, ParamRegistry::ParamType::Double) << param.pointer;
        }
    }
}

TEST(ParamRegistryTable, EnumIndexOfAnUnknownNameIsMinusOne) {
    const ParamRegistry::ParamDef* mode = ParamRegistry::Find("/missile_doctrine/launch_mode");
    ASSERT_NE(mode, nullptr);
    ASSERT_EQ(mode->type, ParamRegistry::ParamType::Enum);
    EXPECT_EQ(ParamRegistry::EnumIndex(*mode, mode->enumAccess.names.front()), 0);
    EXPECT_EQ(ParamRegistry::EnumIndex(*mode, "BOGUS"), -1);
}

// Each doctrine row reaches the field the missile type fills, and the
// type's value sits inside the row's range, so the panel can show it.
TEST(ParamRegistryTable, DoctrineRowsReachTheTypesValuesInRange) {
    ScenarioData scenario;
    scenario.SetMissileType(MissileCatalog::Default().Get("aim120"));
    World world;
    for (const ParamRegistry::ParamDef& param : ParamRegistry::All()) {
        if (param.owner != ParamRegistry::ParamOwner::Doctrine) continue;
        switch (param.type) {
            case ParamRegistry::ParamType::Double: {
                const double* value = param.dbl(scenario, world, kNoEntity);
                ASSERT_NE(value, nullptr) << param.pointer;
                EXPECT_TRUE(std::isfinite(*value)) << param.pointer;
                EXPECT_GE(*value, param.minValue) << param.pointer;
                EXPECT_LE(*value, param.maxValue) << param.pointer;
                break;
            }
            case ParamRegistry::ParamType::Bool:
                EXPECT_NE(param.flag(scenario, world, kNoEntity), nullptr) << param.pointer;
                break;
            case ParamRegistry::ParamType::Enum:
                EXPECT_GE(param.enumAccess.get(scenario, world, kNoEntity), 0) << param.pointer;
                break;
        }
    }
}
