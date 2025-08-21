//
// Created by Andres Guerrero on 16-08-25.
//

#pragma once

#include <type_traits>

#include "Resource.h"

namespace engine::resources {

    // Type-safe resource handle
    template<typename T>
    class ResourceHandle {
        static_assert(std::is_base_of_v<Resource, T>, "T must derive from Resource");

    public:
        ResourceHandle() = default;
        explicit ResourceHandle(const ResourceID id) : id_(id) {}
        ResourceHandle(const ResourceID id, WeakResourcePtr<T> resource) : id_(id), cachedResource_(resource) {}

        // Check if handle is valid
        bool isValid() const { return id_ != INVALID_RESOURCE_ID; }
        explicit operator bool() const { return isValid(); }

        // Get resource ID
        ResourceID getId() const { return id_; }

        // Get actual resource (may trigger loading)
        ResourcePtr<T> get() const;
        ResourcePtr<T> operator->() const { return get(); }
        T& operator*() const { return *get(); }

        // Try to get without loading
        ResourcePtr<T> tryGet() const {
            return cachedResource_.lock();
        }

        // Comparison operators
        bool operator==(const ResourceHandle& other) const { return id_ == other.id; }
        bool operator!=(const ResourceHandle& other) const { return id_ != other.id; }
        bool operator<(const ResourceHandle& other) const { return id_ < other.id; }

        // Reset handle
        void reset() {
            id_ = INVALID_RESOURCE_ID;
            cachedResource_.reset();
        }

    private:
        ResourceID id_ = INVALID_RESOURCE_ID;
        mutable WeakResourcePtr<T> cachedResource_;

        friend class ResourceManager;
        void updateCache(WeakResourcePtr<T> resource) const {
            cachedResource_ = resource;
        }
    };

    // Hash function for handles
    template<typename T>
    struct ResourceHandleHash {
        std::size_t operator()(const ResourceHandle<T>& handle) const {
            return std::hash<ResourceID>{}(handle.getId());
        }
    };
}
