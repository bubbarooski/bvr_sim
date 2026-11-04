#include "sim/EventLog.h"

#include <cstdio>

void EventLog::PrintAll() const {
    std::printf("Event log (%zu events):\n", events_.size());
    for (const SimEvent& event : events_) {
        std::printf("  [%7.3fs] %-9s entity=%-3u %s\n", event.simTime, event.type.c_str(), event.entity,
                     event.data.c_str());
    }
}
