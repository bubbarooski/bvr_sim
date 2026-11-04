// The tier an aircraft's radar cone shows (green / amber / red). A radar that
// is scanning is green whether or not it holds a track: the scenarios tell a
// student the cone is "green while it scans, amber once a track is confirmed,
// and goes back to green" when the track drops. Grey would mean the radar is
// not working at all.

#include <gtest/gtest.h>

#include "components/AirborneRadarComponent.h"
#include "components/AirborneRadarTrack.h"
#include "render/RenderSystem.h"
#include "sim/SimContext.h"

namespace {

constexpr EntityID kOwner = 1;
constexpr EntityID kOther = 5;

void AddTrack(SimContext& context, EntityID owner, EntityID target, RadarTrackState state) {
    AirborneRadarTrack& track = context.airborneRadarTracks.FindOrCreate(owner, target);
    track.owner = owner;
    track.target = target;
    track.state = state;
}

RadarTrackMode Tier(SimContext& context, const AirborneRadarComponent& radar = {}) {
    return RenderSystem::RadarConeTier(context, kOwner, radar);
}

}  // namespace

TEST(RadarConeTier, AScanningRadarThatHoldsNothingIsGreen) {
    SimContext context;
    EXPECT_EQ(Tier(context), RadarTrackMode::SEARCH);
}

TEST(RadarConeTier, ATrackThatWasDroppedGoesBackToGreen) {
    SimContext context;
    AddTrack(context, kOwner, 2, RadarTrackState::CONFIRMED);
    EXPECT_EQ(Tier(context), RadarTrackMode::TRACK);
    AddTrack(context, kOwner, 2, RadarTrackState::NONE);  // dropped after missed frames
    EXPECT_EQ(Tier(context), RadarTrackMode::SEARCH);
}

TEST(RadarConeTier, TheMostAdvancedThingTheRadarHoldsWins) {
    SimContext context;
    AddTrack(context, kOwner, 2, RadarTrackState::TENTATIVE);
    EXPECT_EQ(Tier(context), RadarTrackMode::SEARCH);
    AddTrack(context, kOwner, 3, RadarTrackState::CONFIRMED);
    EXPECT_EQ(Tier(context), RadarTrackMode::TRACK);
    AirborneRadarComponent locked;
    locked.sttTarget = 3;
    EXPECT_EQ(Tier(context, locked), RadarTrackMode::FIRING_SOLUTION);
}

TEST(RadarConeTier, AnotherRadarsTracksDoNotCount) {
    SimContext context;
    AddTrack(context, kOther, 2, RadarTrackState::CONFIRMED);
    EXPECT_EQ(Tier(context), RadarTrackMode::SEARCH);
}
