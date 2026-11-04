#include <gtest/gtest.h>

#include "missile/MissileCatalog.h"
#include "sim/DragTable.h"

TEST(DragTable, LinearInterpolationAtMidpoint) {
    DragTable table({{0.8, 0.35}, {0.9, 0.45}});
    EXPECT_NEAR(table.Lookup(0.85), 0.40, 1e-12);
}

TEST(DragTable, HoldsEndValuesOutsideTheTable) {
    DragTable table({{0.5, 0.30}, {1.0, 0.60}, {4.0, 0.30}});
    EXPECT_EQ(table.Lookup(0.0), 0.30);
    EXPECT_EQ(table.Lookup(6.0), 0.30);
    EXPECT_EQ(table.Lookup(1.0), 0.60);
}

// The AIM-120's table: subsonic-flat, transonic peak, supersonic settle.
TEST(DragTable, Aim120HasTransonicDragBump) {
    const DragTable& drag = MissileCatalog::Default().Get("aim120")->aero.drag;
    double cdSubsonic = drag.Lookup(0.5);
    double cdTransonic = drag.Lookup(1.0);
    double cdSupersonic = drag.Lookup(3.0);
    EXPECT_GT(cdTransonic, cdSubsonic);
    EXPECT_GT(cdTransonic, cdSupersonic);
    EXPECT_GT(drag.Lookup(2.5), cdSubsonic) << "supersonic should remain above subsonic";
}
