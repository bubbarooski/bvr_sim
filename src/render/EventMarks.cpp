#include "render/EventMarks.h"

#include <cstdlib>
#include <string>

#include "components/GuidanceComponent.h"
#include "ecs/ComponentStore.h"

namespace EventMarks {

namespace {

// The radar events name the target as their entity and the radar's owner in
// the data; the tick belongs on the owner's path.
EntityID OwnerOf(const SimEvent& event) {
    std::string owner = EventField(event.data, "owner");
    return owner.empty() ? kNoEntity : static_cast<EntityID>(std::strtoul(owner.c_str(), nullptr, 10));
}

std::optional<MarkSpec> Mark(EntityID path, std::optional<RadarTrackMode> tier, const char* label) {
    if (path == kNoEntity) return std::nullopt;
    return MarkSpec{path, tier, label};
}

}  // namespace

std::optional<MarkSpec> Classify(World& world, const SimEvent& event) {
    const std::string& type = event.type;
    if (type == "RADAR_TRACK_ESTABLISHED") return Mark(OwnerOf(event), RadarTrackMode::TRACK, "Radar track");
    if (type == "RADAR_LOCK") return Mark(OwnerOf(event), RadarTrackMode::FIRING_SOLUTION, "Radar lock");
    // Released by fire control: the track is still held, back on the scan.
    if (type == "RADAR_LOCK_RELEASED") return Mark(OwnerOf(event), RadarTrackMode::TRACK, "Lock released");
    // Broken by a dropped track: back to scanning.
    if (type == "RADAR_LOCK_LOST") return Mark(OwnerOf(event), RadarTrackMode::SEARCH, "Lock lost");
    if (type == "LAUNCH") {
        const GuidanceComponent* guidance = world.Components<GuidanceComponent>().TryGet(event.entity);
        return Mark(guidance != nullptr ? guidance->launcher : kNoEntity, std::nullopt, "Missile launched");
    }
    if (type == "RWR_ILLUMINATED") return Mark(event.entity, RadarTrackMode::SEARCH, "RWR: radar");
    if (type == "RWR_LOCK") return Mark(event.entity, RadarTrackMode::FIRING_SOLUTION, "RWR: lock");
    if (type == "RWR_MISSILE") return Mark(event.entity, RadarTrackMode::FIRING_SOLUTION, "RWR: missile");
    // The seeker's tiers as its cone shows them (RenderSystem's SeekerCueMode).
    if (type == "SEEKER_ACTIVE") return Mark(event.entity, RadarTrackMode::FIRING_SOLUTION, "Seeker active");
    if (type == "SEEKER_LOST") {
        bool onDatalink = EventField(event.data, "fallback") == "datalink";
        return Mark(event.entity, onDatalink ? RadarTrackMode::SEARCH : RadarTrackMode::TRACK,
                    onDatalink ? "Seeker lost: back on datalink" : "Seeker lost: coasting");
    }
    if (type == "LOFT_END") return Mark(event.entity, std::nullopt, "Loft ends");
    if (type == "DATALINK_LOST") return Mark(event.entity, std::nullopt, "Datalink lost");
    return std::nullopt;
}

}  // namespace EventMarks
