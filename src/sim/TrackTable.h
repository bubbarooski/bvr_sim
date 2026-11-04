#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ecs/World.h"

// Per-pair radar tracks (keyed by owner + target) with an O(1)
// hashed lookup beside the backing vector. The vector gives stable
// first-seen iteration order; the hash is only point-queried, so its order
// is never observed and determinism is unaffected.
//
// T must provide a `TrackKey(const T&)` returning its {owner, target} pair.

inline uint64_t PackTrackKey(EntityID a, EntityID b) {
    return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
}

template <typename T>
class TrackTable {
public:
    // Returns the existing track for this pair, or default-constructs,
    // appends, and indexes a new one. The caller sets the track's own key
    // fields (owner, target) -- idempotent for an existing entry.
    T& FindOrCreate(EntityID a, EntityID b) {
        uint64_t key = PackTrackKey(a, b);
        auto it = index_.find(key);
        if (it != index_.end()) return tracks_[it->second];
        index_.emplace(key, tracks_.size());
        tracks_.emplace_back();
        return tracks_.back();
    }

    // O(1) point lookup; nullptr if this pair has no entry yet.
    const T* Find(EntityID a, EntityID b) const {
        auto it = index_.find(PackTrackKey(a, b));
        return it != index_.end() ? &tracks_[it->second] : nullptr;
    }

    // Appends and indexes in one step (manual setup in tests). Reads the
    // pair from the track's own fields.
    void push_back(const T& track) {
        std::pair<EntityID, EntityID> key = TrackKey(track);
        auto it = index_.find(PackTrackKey(key.first, key.second));
        if (it != index_.end()) {
            tracks_[it->second] = track;
            return;
        }
        index_.emplace(PackTrackKey(key.first, key.second), tracks_.size());
        tracks_.push_back(track);
    }

    void clear() {
        tracks_.clear();
        index_.clear();
    }

    bool empty() const { return tracks_.empty(); }
    std::size_t size() const { return tracks_.size(); }
    T& operator[](std::size_t i) { return tracks_[i]; }
    const T& operator[](std::size_t i) const { return tracks_[i]; }

    typename std::vector<T>::iterator begin() { return tracks_.begin(); }
    typename std::vector<T>::iterator end() { return tracks_.end(); }
    typename std::vector<T>::const_iterator begin() const { return tracks_.begin(); }
    typename std::vector<T>::const_iterator end() const { return tracks_.end(); }

private:
    std::vector<T> tracks_;
    std::unordered_map<uint64_t, std::size_t> index_;
};
