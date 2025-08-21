//
// Created by Andres Guerrero on 18-08-25.
//

#pragma once

#include <string>
#include <unordered_set>
#include <utility>

#include "../core/ResourceTypes.h"
#include "../manager/ResourceManager.h"

namespace engine::resources {
    // Resource group for batch loading/unloading
    class ResourceGroup {
    public:
        explicit ResourceGroup(std::string name) : name_(std::move(name)) {
        }

        // Add resources to group
        void addResource(const ResourceID id) {
            std::unique_lock lock(mutex_);
            resources_.insert(id);
        }

        template <typename T>
        void addResource(const ResourceHandle<T>& handle) {
            addResource(handle.getId());
        }

        // Remove resources from group
        void removeResource(const ResourceID id) {
            std::unique_lock lock(mutex_);
            resources_.erase(id);
        }

        // Load/unload all resources in group
        void loadAll(ResourceManager* manager, ResourcePriority priority = ResourcePriority::NORMAL);
        void unloadAll(ResourceManager* manager);

        // Query
        bool contains(const ResourceID id) const {
            std::shared_lock lock(mutex_);

            return resources_.contains(id);
        }

        std::size_t size() const {
            std::shared_lock lock(mutex_);

            return resources_.size();
        }

        const std::string& getName() const { return name_; }

        // Getall resource IDs
        std::vector<ResourceID> getResources() const {
            std::shared_lock lock(mutex_);

            return {resources_.begin(), resources_.end()};
        }

    private:
        std::string name_;
        std::unordered_set<ResourceID> resources_;
        mutable std::shared_mutex mutex_;

        // Loading state
        std::atomic<bool> isLoading_{false};
        std::atomic<std::size_t> loadedCount_{0};
    };
}
