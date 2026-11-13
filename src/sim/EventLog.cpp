#include "sim/EventLog.h"

#include <cstdio>

std::string EventField(const std::string& data, const std::string& key) {
    std::string needle = key + "=";
    size_t pos = 0;
    while ((pos = data.find(needle, pos)) != std::string::npos) {
        if (pos == 0 || data[pos - 1] == ' ') {
            size_t start = pos + needle.size();
            return data.substr(start, data.find(' ', start) - start);
        }
        pos += needle.size();
    }
    return "";
}

void EventLog::PrintAll() const {
    std::printf("Event log (%zu events):\n", events_.size());
    for (const SimEvent& event : events_) {
        std::printf("  [%7.3fs] %-9s entity=%-3u %s\n", event.simTime, event.type.c_str(), event.entity,
                     event.data.c_str());
    }
}
