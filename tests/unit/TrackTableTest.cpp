#include <gtest/gtest.h>

#include "components/AirborneRadarTrack.h"
#include "sim/TrackTable.h"

TEST(TrackTable, FindOnMissingPairReturnsNull) {
    TrackTable<AirborneRadarTrack> table;
    EXPECT_EQ(table.Find(1, 2), nullptr);
}

TEST(TrackTable, FindOrCreateIsIdempotent) {
    TrackTable<AirborneRadarTrack> table;
    AirborneRadarTrack& first = table.FindOrCreate(1, 2);
    first.lastSeenTime = 0.5;
    AirborneRadarTrack& second = table.FindOrCreate(1, 2);
    EXPECT_EQ(&first, &second);
    EXPECT_EQ(second.lastSeenTime, 0.5);
    EXPECT_EQ(table.size(), 1u);
}

TEST(TrackTable, FindLocatesAnIndexedEntry) {
    TrackTable<AirborneRadarTrack> table;
    table.FindOrCreate(10, 20).lastSeenTime = 0.75;
    const AirborneRadarTrack* found = table.Find(10, 20);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->lastSeenTime, 0.75);
}

TEST(TrackTable, PairOrderIsNotInterchangeable) {
    TrackTable<AirborneRadarTrack> table;
    table.FindOrCreate(1, 2).lastSeenTime = 0.1;
    EXPECT_EQ(table.Find(2, 1), nullptr);
}

TEST(TrackTable, PushBackIndexesFromTheTracksOwnFields) {
    TrackTable<AirborneRadarTrack> table;
    AirborneRadarTrack track;
    track.owner = 5;
    track.target = 6;
    track.lastSeenTime = 0.9;
    table.push_back(track);

    const AirborneRadarTrack* found = table.Find(5, 6);
    ASSERT_NE(found, nullptr);
    EXPECT_EQ(found->lastSeenTime, 0.9);
}

// push_back is how tests seed a table. A pair that is already there is
// updated in place: appending a second copy left Find returning the first and
// iteration yielding both.
TEST(TrackTable, PushBackOfAnExistingPairReplacesItInPlace) {
    TrackTable<AirborneRadarTrack> table;
    AirborneRadarTrack track;
    track.owner = 5;
    track.target = 6;
    track.lastSeenTime = 0.9;
    table.push_back(track);
    track.lastSeenTime = 1.5;
    table.push_back(track);

    EXPECT_EQ(table.size(), 1u);
    ASSERT_NE(table.Find(5, 6), nullptr);
    EXPECT_EQ(table.Find(5, 6)->lastSeenTime, 1.5);
}

TEST(TrackTable, IterationPreservesFirstSeenOrder) {
    TrackTable<AirborneRadarTrack> table;
    table.FindOrCreate(1, 1).owner = 1;
    table.FindOrCreate(2, 2).owner = 2;
    table.FindOrCreate(3, 3).owner = 3;

    std::vector<EntityID> order;
    for (const AirborneRadarTrack& t : table) order.push_back(t.owner);
    ASSERT_EQ(order.size(), 3u);
    EXPECT_EQ(order[0], 1u);
    EXPECT_EQ(order[1], 2u);
    EXPECT_EQ(order[2], 3u);
}

TEST(TrackTable, ClearEmptiesBothTheVectorAndTheIndex) {
    TrackTable<AirborneRadarTrack> table;
    table.FindOrCreate(1, 2);
    table.clear();
    EXPECT_TRUE(table.empty());
    EXPECT_EQ(table.size(), 0u);
    EXPECT_EQ(table.Find(1, 2), nullptr);
}
