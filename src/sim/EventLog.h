#pragma once

#include <string>
#include <vector>

#include "ecs/World.h"

// One recorded engagement-timeline event. `type` is a short human-readable
// string ("LAUNCH", "INTERCEPT", "MISS") rather than an enum.
struct SimEvent {
    double simTime = 0.0;
    std::string type;
    EntityID entity = 0;
    std::string data;
};

// The value of `key=` in an event's space-separated data, or "" if absent.
std::string EventField(const std::string& data, const std::string& key);

// Human-readable event log, owned by SimContext alongside the seeded RNG
// stream. Systems call Record directly. This is what the GUI renders and
// what a headless batch run parses for outcome statistics.
class EventLog {
public:
    void Record(SimEvent event) { events_.push_back(std::move(event)); }
    const std::vector<SimEvent>& Events() const { return events_; }
    void Clear() {
        events_.clear();
        ++generation_;
    }
    // Changes whenever Clear() starts a new run's log. A viewer that caches
    // the events compares this, not the size: the new log may already be as
    // long as the old one when it next looks.
    unsigned long long Generation() const { return generation_; }

    // Console dump of every recorded event in order.
    void PrintAll() const;

private:
    std::vector<SimEvent> events_;
    unsigned long long generation_ = 0;
};
