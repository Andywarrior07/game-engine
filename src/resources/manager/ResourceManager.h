//
// Created by Andres Guerrero on 18-08-25.
//

#pragma once

#include <future>
#include <string>
#include <vector>

#include "ResourceFactory.h"
#include "../../memory/MemorySystem.h"
#include "../cache/ResourceCache.h"
#include "../core/Resource.h"
#include "../core/ResourceTypes.h"
#include "../core/ResourceHandle.h"

namespace engine::resources {
    struct ResourceManagerConfig {
        // Memory settings
        memory::MemorySize maxMemory = 512 * 1024 * 1024; // 512 MB
        memory::MemorySize cacheMemory = 128 * 1024 * 1024; // 128 MB

        // Threading
        std::size_t loaderThreadCount = 4;
        std::size_t processorThreadCount = 4;

        // Paths
        std::vector<std::string> searchPaths;
        std::string cachePath = "/cache";

        // Behavior
        bool enableHotReload = true;
        bool enableAsyncLoading = true;
        bool enableMemoryMapping = true;
        bool enableCompression = true;
        bool validateChecksums = true;

        // Limits
        std::size_t maxPendingLoads = 100;
        std::size_t maxConcurrentLoads = 10;
        std::chrono::milliseconds loadTimeout{30000}; // 30 seconds

        // Garbage collection
        std::chrono::seconds gcInterval{60}; // Run GC every minute
        float gcMemoryThreshold = 0.9f; // Run GC at 90% memory usage
    };

    // Load request for async loading
    struct LoadRequest {
        ResourceID id = INVALID_RESOURCE_ID;
        std::string path;
        ResourceType type = ResourceType::UNKNOWN;
        ResourcePriority priority;
        LoadMode mode;
        std::promise<ResourcePtr<Resource>> promise;
        std::function<void(ResourcePtr<Resource>)> callback;
    };

    // Main resource manager class
    class ResourceManager {
    public:
        explicit ResourceManager(memory::MemoryManager* memoryManager);
        ~ResourceManager();

        // Delete copy, allow move
        ResourceManager(const ResourceManager&) = delete;
        ResourceManager& operator=(const ResourceManager&) = delete;
        ResourceManager(ResourceManager&&) = default;
        ResourceManager& operator=(ResourceManager&&) = default;

        // Initialization
        bool initialize(const ResourceManagerConfig& config);
        void shutdown();

        // Resource registration
        template<typename T>
        void registerResourceType() {
            factory_.registerType<T>(T::getStaticType());
        }

        void registerLoader(std::unique_ptr<IResourceLoader> loader);

        // Resource loading
        template<typename T>
        ResourceHandle<T> loadById(ResourceID id,
            std::string path,
            const ResourcePriority priority = ResourcePriority::NORMAL,
            const LoadMode mode = LoadMode::ASYNC) {
            // Check if already loaded
            if (auto resource = getResource<T>(id)) {
                return ResourceHandle<T>(id, resource);
            }

            // Get resource type
            auto typeOpt = factory_.getResourceType<T>();
            if (!typeOpt) {
                return ResourceHandle<T>();
            }

            // Create load request
            if (mode == LoadMode::ASYNC && config_.enableAsyncLoading) {
                auto future = loadAsync(id, path, *typeOpt, priority);
                // Store future for later retrieval
                pendingLoads_[id] = std::move(future);
            } else {
                loadSync(id, path, *typeOpt);
            }

            return ResourceHandle<T>(id);
        }

        // Get resource
        template<typename T>
        ResourcePtr<T> getResource(ResourceID id) {
            std::shared_lock lock(resourceMutex_);

            if (auto it = resources_.find(id); it != resources_.end()) {
                return std::dynamic_pointer_cast<T>(it->second);
            }

            return nullptr;
        }

        template<typename T>
        ResourcePtr<T> getResource(const std::string& name) {
            const ResourceID id = generateResourceID(name);

            return getResource<T>(id);
        }

        // Resource unloading
        void unload(ResourceID id);
        void unloadAll();
        void unloadByType(ResourceType type);

        // Memory management
        void garbageCollect();
        void purgeCache();
        ResourceSize getMemoryUsage() const;
        ResourceSize getCacheMemoryUsage() const;

        // Hot reload
        void checkForChanges();
        void reloadResource(ResourceID id);
        void enableHotReload(const bool enable) { config_.enableHotReload = enable; }

        // Statistics
        const ResourceStats& getStats() const { return stats_; }
        std::string generateReport() const;

        // Callbacks
        using LoadCallback = std::function<void(ResourceID, bool)>;
        void setLoadCallback(const LoadCallback callback) { loadCallback_ = callback; }

    private:
        // Configuration
        ResourceManagerConfig config_;

        memory::MemoryManager* memoryManager_;

        // Resources
        std::unordered_map<ResourceID, ResourcePtr<Resource>> resources_;
        mutable std::shared_mutex resourceMutex_;

        // Factory and loaders
        ResourceFactory factory_;
        std::vector<std::unique_ptr<IResourceLoader>> loaders_;

        // Caches
        std::unordered_map<ResourceType, std::unique_ptr<ResourceCache<Resource>>> caches_;

        // Async loading
        std::queue<LoadRequest> loadQueue_;
        std::mutex queueMutex_;
        std::condition_variable queueCV_;
        std::vector<std::thread> loaderThreads_;
        std::atomic<bool> stopLoading_{false};

        // Pending loads
        std::unordered_map<ResourceID, std::future<ResourcePtr<Resource>>> pendingLoads_;
        std::mutex pendingMutex_;

        // Hot reload
        std::thread hotReloadThread_;
        std::atomic<bool> stopHotReload_{false};
        std::unordered_map<std::string, std::chrono::system_clock::time_point> fileTimestamps_;

        // Garbage collection
        std::thread gcThread_;
        std::atomic<bool> stopGC_{false};

        // statistics
        mutable ResourceStats stats_;

        // Callbacks
        LoadCallback loadCallback_;

        // Helper methods
        ResourceID generateResourceID(const std::string& path) const;
        IResourceLoader* selectLoader(const std::string& path, ResourceType type) const;

        // Loading implementation
        ResourcePtr<Resource> loadSync(ResourceID id, std::string path, ResourceType type);
        std::future<ResourcePtr<Resource>> loadAsync(ResourceID id, const std::string& path,
                                                     ResourceType type, ResourcePriority priority);
        void loadThreadWorker();

        // Hot reload implementation
        void hotReloadWorker();
        bool hasFileChanged(const std::string& path);

        // Garbage collection implementation
        void gcWorker();
        bool shouldRunGC() const;

        // Memory allocation
        void* allocateResourceMemory(ResourceSize size, memory::MemoryCategory category) const;
        void deallocateResourceMemory(void* ptr, memory::MemoryCategory category) const;
    };
}
