#include "World.h"

#include <utility>

EntityID World::CreateEntity() {
    EntityID id = nextEntityId_++;
    entities_.insert(id);
    return id;
}

void World::Clear() {
    entities_.clear();
    nextEntityId_ = 1;
    for (auto& [type, store] : stores_) {
        store->Clear();
    }
}

void World::ReplaceWith(World&& other) {
    entities_ = std::move(other.entities_);
    nextEntityId_ = other.nextEntityId_;
    for (auto& [type, store] : stores_) {
        auto it = other.stores_.find(type);
        if (it == other.stores_.end()) {
            store->Clear();
        } else {
            store->TakeFrom(*it->second);
        }
    }
    // A type only `other` has moves across whole. Erase its entry so `other`
    // never holds a moved-from (null) store that its own Clear() or a later
    // Components<T>() would dereference.
    for (auto it = other.stores_.begin(); it != other.stores_.end();) {
        if (stores_.find(it->first) == stores_.end()) {
            stores_.emplace(it->first, std::move(it->second));
            it = other.stores_.erase(it);
        } else {
            ++it;
        }
    }
    other.Clear();
}
