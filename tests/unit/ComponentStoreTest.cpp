// Systems iterate a ComponentStore and several draw from the one seeded RNG
// as they go, so the order they visit entities in is part of what a seed
// means. That order is ascending EntityID, the same on every platform and
// whatever order the components were added in; a hash container would visit
// them in a library-specific order and give MSVC and libstdc++ different runs.

#include <vector>

#include <gtest/gtest.h>

#include "components/Tag.h"
#include "components/Velocity.h"
#include "ecs/ComponentStore.h"
#include "ecs/World.h"

namespace {

std::vector<EntityID> IterationOrder(ComponentStore<Velocity>& store) {
    std::vector<EntityID> order;
    for (auto& [entity, velocity] : store) order.push_back(entity);
    return order;
}

}  // namespace

TEST(ComponentStore, IteratesInAscendingEntityIdWhateverTheInsertionOrder) {
    ComponentStore<Velocity> store;
    for (EntityID entity : {5u, 2u, 9u, 1u, 7u, 3u}) store.Add(entity, Velocity{});
    EXPECT_EQ(IterationOrder(store), (std::vector<EntityID>{1, 2, 3, 5, 7, 9}));
}

TEST(ComponentStore, AddingNeverMovesAComponentAlreadyHeld) {
    ComponentStore<Velocity> store;
    Velocity& first = store.Add(4, Velocity{Vec3(1.0, 2.0, 3.0)});
    for (EntityID entity = 5; entity < 200; ++entity) store.Add(entity, Velocity{});
    store.Add(1, Velocity{});
    EXPECT_EQ(&first, &store.Get(4));
    EXPECT_EQ(first.linear.y(), 2.0);
}

TEST(ComponentStore, TryGetAndHasAgreeAboutPresence) {
    ComponentStore<Velocity> store;
    store.Add(3, Velocity{});
    EXPECT_TRUE(store.Has(3));
    EXPECT_NE(store.TryGet(3), nullptr);
    EXPECT_FALSE(store.Has(4));
    EXPECT_EQ(store.TryGet(4), nullptr);
    EXPECT_EQ(store.Size(), 1u);
}

TEST(World, EntitiesAreListedInAscendingOrder) {
    World world;
    for (int i = 0; i < 12; ++i) world.CreateEntity();
    EntityID previous = 0;
    for (EntityID entity : world.Entities()) {
        EXPECT_GT(entity, previous);
        previous = entity;
    }
    EXPECT_EQ(world.Entities().size(), 12u);
}
