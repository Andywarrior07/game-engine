//
// Created by Andres Guerrero on 18-08-25.
//

#include "ResourceManager.h"

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <ranges>

#include "../loader/IResourceLoader.h"

namespace engine::resources {
    ResourceManager::ResourceManager(memory::MemoryManager* memoryManager)
        : memoryManager_(memoryManager) {
    }

    ResourceManager::~ResourceManager() {
        shutdown();
    }

    bool ResourceManager::initialize(const ResourceManagerConfig& requestedConfig) {
        config_ = requestedConfig;

        try {
            // Register default loader
            registerLoader(std::make_unique<FileSystemLoader>());

            // Initialize caches for each resource type
            for (std::size_t i = 0; i < static_cast<std::size_t>(ResourceType::COUNT); i++) {
                auto cache = std::make_unique<ResourceCache<Resource>>(
                    config_.cacheMemory / static_cast<std::size_t>(ResourceType::COUNT),
                    100 // Max 100 resources per type in cache
                );

                caches_[static_cast<ResourceType>(i)] = std::move(cache);
            }

            // Start loader threads
            if (config_.enableAsyncLoading) {
                stopLoading_ = false;
                for (std::size_t i = 0; i < config_.loaderThreadCount; i++) {
                    loaderThreads_.emplace_back(&ResourceManager::loadThreadWorker, this);
                }
            }

            // Start hot reload thread
            if (config_.enableHotReload) {
                stopHotReload_ = false;
                hotReloadThread_ = std::thread(&ResourceManager::hotReloadWorker, this);
            }

            // Start garbage collection thread
            stopGC_ = false;
            gcThread_ = std::thread(&ResourceManager::gcWorker, this);

            std::cout << "ResourceManager initialized successfully:" << std::endl;
            std::cout << "  Max Memory: " << (config_.maxMemory / (1024.0 * 1024.0)) << " MB" << std::endl;
            std::cout << "  Cache Memory: " << (config_.cacheMemory / (1024.0 * 1024.0)) << " MB" << std::endl;
            std::cout << "  Loader Threads: " << config_.loaderThreadCount << std::endl;
            std::cout << "  Async Loading: " << (config_.enableAsyncLoading ? "Enabled" : "Disabled") << std::endl;
            std::cout << "  Hot Reload: " << (config_.enableHotReload ? "Enabled" : "Disabled") << std::endl;

            return true;
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to initialize ResourceManager: " << e.what() << std::endl;
            shutdown();
            return false;
        }
    }

    void ResourceManager::shutdown() {
        // Stop all threads
        stopLoading_ = true;
        stopHotReload_ = true;
        stopGC_ = true;

        // Wake up threads
        queueCV_.notify_all();

        // Join threads
        for (auto& thread : loaderThreads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }

        if (hotReloadThread_.joinable()) {
            hotReloadThread_.join();
        }

        if (gcThread_.joinable()) {
            gcThread_.join();
        }

        // Clear all resources
        unloadAll();

        // Clear caches
        caches_.clear();

        // Clear loaders
        loaders_.clear();

        std::cout << "ResourceManager shutdown successfully" << std::endl;
    }

    void ResourceManager::registerLoader(std::unique_ptr<IResourceLoader> loader) {
        if (!loader) return;

        // Insert loader sorted by priority (higher priority first)
        const auto it = std::ranges::find_if(loaders_,
                                             [&loader](const std::unique_ptr<IResourceLoader>& existing) {
                                                 return existing->getPriority() < loader->getPriority();
                                             });

        loaders_.insert(it, std::move(loader));
    }

    void ResourceManager::unload(const ResourceID id) {
        std::unique_lock lock(resourceMutex_);

        const auto it = resources_.find(id);

        if (it == resources_.end()) {
            return;
        }

        auto& resource = it->second;

        // Check reference count
        if (resource->getReferenceCount() > 0) {
            std::cerr << "Warning: Unloading resource with active references: " << resource->getName() << std::endl;
        }

        // Update stats
        stats_.totalMemoryUsage.fetch_sub(resource->getMemoryUsage(), std::memory_order_relaxed);
        stats_.loadedResourceCount.fetch_sub(1, std::memory_order_relaxed);

        // Unload the resource
        resource->unload();

        // Remove from cache
        if (const auto cache = caches_[resource->getType()].get()) {
            cache->remove(id);
        }

        // Remove from map
        resources_.erase(it);
    }

    void ResourceManager::unloadAll() {
        std::unique_lock lock(resourceMutex_);

        for (const auto& resource : resources_ | std::views::values) {
            resource->unload();
        }

        resources_.clear();

        // Clear all caches
        for (auto& cache : caches_ | std::views::values) {
            if (cache) {
                cache->clear();
            }
        }

        // Reset stats
        stats_.totalMemoryUsage = 0;
        stats_.loadedResourceCount = 0;
    }

    void ResourceManager::unloadByType(ResourceType type) {
        std::unique_lock lock(resourceMutex_);

        auto it = resources_.begin();

        while (it != resources_.end()) {
            if (it->second->getType() == type) {
                it->second->unload();

                // Update stats
                stats_.totalMemoryUsage.fetch_sub(it->second->getMemoryUsage(), std::memory_order_relaxed);
                stats_.loadedResourceCount.fetch_sub(1, std::memory_order_relaxed);

                it = resources_.erase(it);
            }
            else {
                ++it;
            }
        }

        // Clear cache for this type
        if (const auto cache = caches_[type].get()) {
            cache->clear();
        }
    }

    void ResourceManager::garbageCollect() {
        std::cout << "Running garbage collection..." << std::endl;

        std::unique_lock lock(resourceMutex_);

        std::size_t freedCount = 0;
        ResourceSize freedMemory = 0;

        auto it = resources_.begin();
        while (it != resources_.end()) {
            const auto& resource = it->second;

            // Unload resources with zero references that aren't persistent
            if (resource->getReferenceCount() == 0 &&
                !hasFlag(resource->getMetadata().flags, ResourceFlags::PERSISTENT)) {
                freedMemory += resource->getMemoryUsage();
                resource->unload();

                // Remove from cache
                if (const auto cache = caches_[resource->getType()].get()) {
                    cache->remove(resource->getId());
                }

                it = resources_.erase(it);
                ++freedCount;
            }
            else {
                ++it;
            }
        }

        stats_.totalMemoryUsage.fetch_sub(freedMemory, std::memory_order_relaxed);
        stats_.loadedResourceCount.fetch_sub(freedCount, std::memory_order_relaxed);

        std::cout << "Garbage collection complete: freed " << freedCount
            << " resources (" << (freedMemory / 1024.0) << " KB)" << std::endl;
    }

    void ResourceManager::purgeCache() {
        for (auto& cache : caches_ | std::views::values) {
            if (cache) {
                cache->clear();
            }
        }

        std::cout << "All resource caches purged." << std::endl;
    }

    ResourceSize ResourceManager::getMemoryUsage() const {
        return stats_.totalMemoryUsage.load(std::memory_order_relaxed);
    }

    ResourceSize ResourceManager::getCacheMemoryUsage() const {
        ResourceSize total = 0;

        for (const auto& cache : caches_ | std::views::values) {
            if (cache) {
                const auto stats = cache->getStats();
                total += stats.memoryUsage;
            }
        }

        return total;
    }

    void ResourceManager::checkForChanges() {
        if (!config_.enableHotReload) return;

        std::shared_lock lock(resourceMutex_);

        for (const auto& [id, resource] : resources_) {
            if (resource->supportsHotReload()) {
                const auto& path = resource->getMetadata().path;

                if (hasFileChanged(path)) {
                    // Queue for reload
                    reloadResource(id);
                }
            }
        }
    }

    void ResourceManager::reloadResource(const ResourceID id) {
        const auto resource = getResource<Resource>(id);

        if (!resource) return;

        std::cout << "Hot reloading resource: " << resource->getName() << std::endl;

        // Store metadata
        const auto metadata = resource->getMetadata();

        // Reload
        resource->reload();

        // Reload from disk
        if (const auto loader = selectLoader(metadata.path, metadata.type)) {
            ResourceDataSource source;
            source.path = metadata.path;

            if (const auto result = loader->load(source); result.success) {
                resource->load(result.data.get(), result.size);
                resource->setState(ResourceState::READY);
                resource->onHotReload();

                ++stats_.hotReloads;
            }
        }
    }

    std::string ResourceManager::generateReport() const {
        std::stringstream report;

        report << "=== Resource Manager Report ===" << std::endl;
        report << std::endl;

        // Overall statistics
        report << "Overall Statistics:" << std::endl;
        report << "  Total Resources: " << stats_.totalResourceCount.load() << std::endl;
        report << "  Loaded Resources: " << stats_.loadedResourceCount.load() << std::endl;
        report << "  Memory Usage: " << (getMemoryUsage() / (1024.0 * 1024.0)) << " MB" << std::endl;
        report << "  Cache Memory: " << (getCacheMemoryUsage() / (1024.0 * 1024.0)) << " MB" << std::endl;
        report << "  Failed Loads: " << stats_.failedLoads.load() << std::endl;
        report << "  Hot Reloads: " << stats_.hotReloads.load() << std::endl;
        report << std::endl;

        // Cache statistics
        report << "Cache Statistics:" << std::endl;
        for (const auto& [type, cache] : caches_) {
            if (cache) {
                auto cacheStats = cache->getStats();
                report << "  Type " << static_cast<int>(type) << ": "
                    << "Hits=" << cacheStats.hitCount
                    << ", Misses=" << cacheStats.missCount
                    << ", Rate=" << std::fixed << std::setprecision(2)
                    << (cacheStats.hitRate * 100.0f) << "%"
                    << ", Memory=" << (cacheStats.memoryUsage / 1024.0) << " KB"
                    << std::endl;
            }
        }
        report << std::endl;

        // Per-type breakdown
        report << "Resource Type Breakdown:" << std::endl;
        const char* typeNames[] = {
            "UNKNOWN", "TEXTURE", "MESH", "SHADER", "MATERIAL",
            "ANIMATION", "AUDIO", "FONT", "SCRIPT", "LEVEL",
            "PREFAB", "PHYSICS_MAT", "PARTICLES", "UI", "CONFIG"
        };

        for (std::size_t i = 0; i < static_cast<std::size_t>(ResourceType::COUNT); ++i) {
            if (const std::size_t count = stats_.typeCount[i].load(); count > 0) {
                const ResourceSize memory = stats_.typeMemory[i].load();
                report << "  " << std::setw(15) << typeNames[i] << ": "
                    << std::setw(5) << count << " resources, "
                    << std::setw(10) << (memory / (1024.0 * 1024.0)) << " MB" << std::endl;
            }
        }

        return report.str();
    }

    ResourceID ResourceManager::generateResourceID(const std::string& path) const {
        // Simple hash function for resource ID generation
        // In production, use a better hash like xxHash or CityHash
        constexpr std::hash<std::string> hasher;
        return hasher(path);
    }

    IResourceLoader* ResourceManager::selectLoader(const std::string& path, ResourceType type) const {
        for (const auto& loader : loaders_) {
            if (loader->canLoad(path, type)) {
                return loader.get();
            }
        }

        return nullptr;
    }

    ResourcePtr<Resource> ResourceManager::loadSync(ResourceID id, std::string& path, ResourceType type) {
        // Check if already loading
        {
            std::lock_guard lock(pendingMutex_);
            if (pendingLoads_.count(id) > 0) {
                // Wait for pending load
                return pendingLoads_[id].get();
            }
        }

        // Select appropriate loader
        auto loader = selectLoader(path, type);
        if (!loader) {
            std::cerr << "No loader found for resource: " << path << std::endl;
            ++stats_.failedLoads;
            return nullptr;
        }

        // Load resource data
        ResourceDataSource source;
        source.path = path;

        auto loadResult = loader->load(source);
        if (!loadResult.success) {
            std::cerr << "Failed to load resource: " << path << " - " << loadResult.error << std::endl;
            ++stats_.failedLoads;
            return nullptr;
        }

        // Create resource instance
        auto resource = factory_.create(type, id, path);
        if (!resource) {
            std::cerr << "Failed to create resource instance for type: " << static_cast<int>(type) << std::endl;
            ++stats_.failedLoads;
            return nullptr;
        }

        // Initialize resource
        resource->setManager(this);
        resource->metadata_ = loadResult.metadata;
        resource->metadata_.id = id;
        resource->metadata_.type = type;

        // Load resource data
        resource->setState(ResourceState::LOADING);
        if (!resource->load(loadResult.data.get(), loadResult.size)) {
            std::cerr << "Resource failed to process data: " << path << std::endl;
            ++stats_.failedLoads;
            return nullptr;
        }

        resource->setState(ResourceState::READY);

        // Store in manager
        auto sharedResource = std::shared_ptr(std::move(resource));
        {
            std::unique_lock lock(resourceMutex_);
            resources_[id] = sharedResource;
        }

        // Update statistics
        ++stats_.totalResourceCount;
        ++stats_.loadedResourceCount;
        stats_.totalMemoryUsage.fetch_add(sharedResource->getMemoryUsage(), std::memory_order_relaxed);
        stats_.typeCount[static_cast<std::size_t>(type)].fetch_add(1, std::memory_order_relaxed);
        stats_.typeMemory[static_cast<std::size_t>(type)].fetch_add(
            sharedResource->getMemoryUsage(), std::memory_order_relaxed
        );

        // Add to cache
        if (auto cache = caches_[type].get()) {
            cache->insert(id, sharedResource);
        }

        // Trigger callback
        if (loadCallback_) {
            loadCallback_(id, true);
        }

        return sharedResource;
    }

    std::future<ResourcePtr<Resource>> ResourceManager::loadAsync(ResourceID id, const std::string& path,
                                                                  ResourceType type, ResourcePriority priority) {
        std::promise<ResourcePtr<Resource>> promise;
        auto future = promise.get_future();

        LoadRequest request;
        request.id = id;
        request.path = path;
        request.type = type;
        request.priority = priority;
        request.mode = LoadMode::ASYNC;
        request.promise = std::move(promise);

        {
            std::unique_lock lock(queueMutex_);

            // Check queue size limit
            if (loadQueue_.size() >= config_.maxPendingLoads) {
                // Queue is full, fal the request
                request.promise.set_value(nullptr);
                return future;
            }

            loadQueue_.push(std::move(request));
        }

        queueCV_.notify_one();

        return future;
    }

    void ResourceManager::loadThreadWorker() {
        while (!stopLoading_) {
            LoadRequest request;

            {
                std::unique_lock lock(queueMutex_);

                queueCV_.wait(lock, [this] {
                    return !loadQueue_.empty() || stopLoading_;
                });

                if (stopLoading_) break;

                if (!loadQueue_.empty()) {
                    request = std::move(loadQueue_.front());
                    loadQueue_.pop();
                }
            }

            // Process the request
            if (request.id != INVALID_RESOURCE_ID) {
                auto resource = loadSync(request.id, request.path, request.type);
                request.promise.set_value(resource);

                if (request.callback) {
                    request.callback(resource);
                }
            }
        }
    }

    void ResourceManager::hotReloadWorker() {
        while (!stopHotReload_) {
            std::this_thread::sleep_for(std::chrono::seconds(1));

            if (stopHotReload_) break;

            checkForChanges();
        }
    }

    bool ResourceManager::hasFileChanged(const std::string& path) {
        if (!std::filesystem::exists(path)) {
            return false;
        }

        auto lastWriteTime = std::filesystem::last_write_time(path);
        auto lastWriteTimePoint = std::chrono::system_clock::now(); // Simplified conversion

        const auto it = fileTimestamps_.find(path);
        if (it == fileTimestamps_.end()) {
            fileTimestamps_[path] = lastWriteTimePoint;
            return false;
        }

        if (lastWriteTimePoint > it->second) {
            it->second = lastWriteTimePoint;
            return true;
        }

        return false;
    }

    void ResourceManager::gcWorker() {
        while (!stopGC_) {
            std::this_thread::sleep_for(config_.gcInterval);

            if (stopGC_) break;

            if (shouldRunGC()) {
                garbageCollect();
            }
        }
    }

    bool ResourceManager::shouldRunGC() const {
        // Run GC if memory usage exceeds threshold
        const ResourceSize currentUsage = getMemoryUsage();
        const auto threshold = static_cast<ResourceSize>(config_.maxMemory * config_.gcMemoryThreshold);

        return currentUsage > threshold;
    }

    void* ResourceManager::allocateResourceMemory(const ResourceSize size, const memory::MemoryCategory category) const {
        return memoryManager_->allocate(size, category);
    }

    void ResourceManager::deallocateResourceMemory(void* ptr, const memory::MemoryCategory category) const {
        memoryManager_->deallocate(ptr, category);
    }
}
