#pragma once

#include <cstdint>
#include <memory>
#include <set>
#include <typeindex>
#include <unordered_map>

using EntityID = uint32_t;

// IDs start at 1, so 0 never names a real entity -- the "none" sentinel.
constexpr EntityID kNoEntity = 0;

// Type-erased base so World can hold one ComponentStore<T> per component
// type without World.h needing to know about T. The template itself lives
// in ComponentStore.h -- see World::Components() below for why the split.
class IComponentStore {
public:
    virtual ~IComponentStore() = default;
    virtual void Clear() = 0;
    // Replaces this store's contents with `other`'s, which must be a store
    // of the same component type. `other` is left empty.
    virtual void TakeFrom(IComponentStore& other) = 0;
};

template<typename T>
class ComponentStore;  // Defined in ComponentStore.h.

// Owns entity IDs and a lazily-created ComponentStore<T> per component
// type. No component data lives in World itself, only the type-erased map.
class World {
public:
    EntityID CreateEntity();

    // Destroys every entity and clears every component store, resetting
    // entity IDs back to the same sequence a fresh World would produce.
    void Clear();

    // Becomes `other`: its entities, ID sequence, and components. Existing
    // ComponentStore objects are kept and refilled rather than replaced, so
    // references callers hold to them stay valid. Lets a scenario load into
    // a scratch World -- where a failure leaves this one untouched -- and
    // then be swapped in without loading it twice.
    void ReplaceWith(World&& other);

    // Every live entity ID, whatever its components.
    const std::set<EntityID>& Entities() const { return entities_; }

    // Lazily creates the store for T on first access. Declared here but
    // defined in ComponentStore.h: it needs ComponentStore<T> to be a
    // complete type (for make_unique/static_cast), and ComponentStore.h
    // already includes World.h for EntityID/IComponentStore -- defining it
    // here instead would make the two headers include each other.
    template<typename T>
    ComponentStore<T>& Components();

private:
    EntityID nextEntityId_ = 1;
    std::set<EntityID> entities_;
    std::unordered_map<std::type_index, std::unique_ptr<IComponentStore>> stores_;
};
