#include <gtest/gtest.h>

#include <utility>

#include "components/Tag.h"
#include "components/Transform.h"
#include "components/Velocity.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"

TEST(World, EntityIdsStartAtOneAndNeverRepeat) {
    World world;
    EntityID a = world.CreateEntity();
    EntityID b = world.CreateEntity();
    EXPECT_EQ(a, 1u);
    EXPECT_EQ(b, 2u);
    EXPECT_EQ(world.Entities().size(), 2u);
}

TEST(World, ClearRestartsTheIdSequenceAndEmptiesEveryStore) {
    World world;
    EntityID a = world.CreateEntity();
    world.Components<Tag>().Add(a, Tag{"A"});
    world.Clear();
    EXPECT_TRUE(world.Entities().empty());
    EXPECT_EQ(world.Components<Tag>().Size(), 0u);
    EXPECT_EQ(world.CreateEntity(), 1u);
}

TEST(World, ReplaceWithTakesEntitiesIdSequenceAndComponents) {
    World live;
    live.Components<Tag>().Add(live.CreateEntity(), Tag{"old"});

    World fresh;
    EntityID a = fresh.CreateEntity();
    EntityID b = fresh.CreateEntity();
    fresh.Components<Tag>().Add(a, Tag{"A"});
    fresh.Components<Tag>().Add(b, Tag{"B"});

    live.ReplaceWith(std::move(fresh));

    EXPECT_EQ(live.Entities().size(), 2u);
    ASSERT_NE(live.Components<Tag>().TryGet(a), nullptr);
    EXPECT_EQ(live.Components<Tag>().Get(a).name, "A");
    EXPECT_EQ(live.Components<Tag>().Get(b).name, "B");
    EXPECT_EQ(live.CreateEntity(), 3u);
}

// A live world creates a component store only when something first asks for
// it, so a freshly loaded scenario can hold a component type the live world
// has never touched (a custom scenario with a "velocity" block, say).
TEST(World, ReplaceWithAdoptsAComponentTypeTheLiveWorldNeverCreated) {
    World live;
    live.Components<Tag>().Add(live.CreateEntity(), Tag{"old"});

    World fresh;
    EntityID mover = fresh.CreateEntity();
    fresh.Components<Velocity>().Add(mover, Velocity{Vec3(1.0, 2.0, 3.0)});

    live.ReplaceWith(std::move(fresh));

    const Velocity* velocity = live.Components<Velocity>().TryGet(mover);
    ASSERT_NE(velocity, nullptr);
    EXPECT_EQ(velocity->linear.y(), 2.0);
    EXPECT_EQ(live.Components<Tag>().Size(), 0u);
}

TEST(World, ReplacedWorldIsEmptyAndStillUsable) {
    World live;
    World fresh;
    fresh.Components<Velocity>().Add(fresh.CreateEntity(), Velocity{});
    live.ReplaceWith(std::move(fresh));

    EXPECT_TRUE(fresh.Entities().empty());
    // The adopted store left the source; asking for it again must make a
    // working empty one rather than hand back a dangling entry.
    EXPECT_EQ(fresh.Components<Velocity>().Size(), 0u);
    EXPECT_EQ(fresh.CreateEntity(), 1u);
}

TEST(World, ReplaceWithEmptiesATypeTheSourceLacks) {
    World live;
    live.Components<Velocity>().Add(live.CreateEntity(), Velocity{});

    World fresh;
    fresh.Components<Tag>().Add(fresh.CreateEntity(), Tag{"only"});

    live.ReplaceWith(std::move(fresh));
    EXPECT_EQ(live.Components<Velocity>().Size(), 0u);
    EXPECT_EQ(live.Components<Tag>().Size(), 1u);
}

TEST(World, ReplaceWithKeepsReferencesToStoresTheLiveWorldAlreadyHad) {
    World live;
    ComponentStore<Transform>& held = live.Components<Transform>();

    World fresh;
    EntityID a = fresh.CreateEntity();
    fresh.Components<Transform>().Add(a, Transform{Vec3(5.0, 6.0, 7.0), Vec3()});

    live.ReplaceWith(std::move(fresh));
    EXPECT_EQ(&held, &live.Components<Transform>());
    ASSERT_NE(held.TryGet(a), nullptr);
    EXPECT_EQ(held.Get(a).pos.x(), 5.0);
}
