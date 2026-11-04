#pragma once

#include <cstddef>
#include <map>
#include <memory>

#include "World.h"

// One ComponentStore<T> per component type. Systems iterate entities that
// have the components they need and ignore ones that don't.
//
// Iteration is in ascending EntityID, the same on every platform and whatever
// order components were added in. Several systems draw from the one seeded RNG
// as they iterate, so this order is part of what a seed means: a hash
// container would visit entities in a library-specific order. (std::map:
// references and iterators survive an Add.)
//
// A system needing two components iterates one store and TryGets the other
// per entity, skipping on absence:
//   for (auto& [entity, x] : storeA) {
//       Y* y = storeB.TryGet(entity);
//       if (y == nullptr) continue;
//       ...
//   }
// There is no generic join: each system's skip conditions differ enough
// that one would only relocate the branches.
template<typename T>
class ComponentStore : public IComponentStore {
public:
    T& Add(EntityID entity, T component) { return data_[entity] = std::move(component); }
    bool Has(EntityID entity) const { return data_.find(entity) != data_.end(); }
    T& Get(EntityID entity) { return data_.at(entity); }
    const T& Get(EntityID entity) const { return data_.at(entity); }

    // Pointer to the component, or nullptr if absent: one lookup where
    // Has() + Get() costs two.
    T* TryGet(EntityID entity) {
        auto it = data_.find(entity);
        return it != data_.end() ? &it->second : nullptr;
    }
    const T* TryGet(EntityID entity) const {
        auto it = data_.find(entity);
        return it != data_.end() ? &it->second : nullptr;
    }

    std::size_t Size() const { return data_.size(); }

    auto begin() { return data_.begin(); }
    auto end() { return data_.end(); }
    auto begin() const { return data_.begin(); }
    auto end() const { return data_.end(); }

    void Clear() override { data_.clear(); }
    void TakeFrom(IComponentStore& other) override {
        data_ = std::move(static_cast<ComponentStore<T>&>(other).data_);
        static_cast<ComponentStore<T>&>(other).data_.clear();
    }

private:
    std::map<EntityID, T> data_;
};

// Defined here, not in World.h, so the two headers need not include each
// other (see the declaration).
template<typename T>
ComponentStore<T>& World::Components() {
    std::type_index key(typeid(T));
    auto it = stores_.find(key);
    if (it == stores_.end()) {
        auto store = std::make_unique<ComponentStore<T>>();
        ComponentStore<T>* ptr = store.get();
        stores_.emplace(key, std::move(store));
        return *ptr;
    }
    return *static_cast<ComponentStore<T>*>(it->second.get());
}
