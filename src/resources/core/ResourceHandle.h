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
        ResourceHandle(const ResourceID id, ResourceManager* manager)
    : id_(id), resourceManager_(manager) {}
        ResourceHandle(const ResourceID id, WeakResourcePtr<T> resource) : id_(id), cachedResource_(resource) {}

        // Check if handle is valid
        bool isValid() const { return id_ != INVALID_RESOURCE_ID; }
        explicit operator bool() const { return isValid(); }

        // Get resource ID
        ResourceID getId() const { return id_; }

        // Get actual resource (may trigger loading)
        ResourcePtr<T> get() const {
            // First try to get from cache
            if (auto cached = cachedResource_.lock()) {
                return cached;
            }

            // If we have a resource manager, try to get from it
            if (resourceManager_ && id_ != INVALID_RESOURCE_ID) {
                // This requires ResourceManager to have a getResource method
                // We need to include ResourceManager or use a forward declaration
                // and implement this in a .cpp file or after ResourceManager is defined

                // For now, we'll need to make this work differently
                // The ResourceManager should update the cache when loading
                return nullptr;
            }

            return nullptr;
        }
        ResourcePtr<T> operator->() const { return get(); }
        T& operator*() const { return *get(); }

        // Try to get without loading
        ResourcePtr<T> tryGet() const {
            return cachedResource_.lock();
        }

        bool isReady() const {
            auto resource = cachedResource_.lock();
            return resource && resource->isLoaded();
        }

        // Comparison operators
        bool operator==(const ResourceHandle& other) const { return id_ == other.id_; }
        bool operator!=(const ResourceHandle& other) const { return id_ != other.id_; }
        bool operator<(const ResourceHandle& other) const { return id_ < other.id_; }

        // Reset handle
        void reset() {
            id_ = INVALID_RESOURCE_ID;
            cachedResource_.reset();
        }

        friend class ResourceManager;
        void updateCache(WeakResourcePtr<T> resource) const {
            cachedResource_ = resource;
        }
    private:
        ResourceID id_ = INVALID_RESOURCE_ID;
        mutable WeakResourcePtr<T> cachedResource_;
        ResourceManager* resourceManager_ = nullptr;
    };

    // Hash function for handles
    template<typename T>
    struct ResourceHandleHash {
        std::size_t operator()(const ResourceHandle<T>& handle) const {
            return std::hash<ResourceID>{}(handle.getId());
        }
    };
}
