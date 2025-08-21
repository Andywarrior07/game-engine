// //
// // Created by Andres Guerrero on 05-08-25.
// //
//
// #include "ResourceManager.h"
// #include <filesystem>           // For file system operations (C++17)
// #include <algorithm>           // For std::sort, std::remove_if, etc.
// #include <cassert>             // For debug assertions
// #include <fstream>             // For file existence checking
// #include <iostream>            // For debug output
// #ifdef _WIN32
//     #include <windows.h>       // For Windows-specific memory info
// #elif defined(__APPLE__)
//     #include <mach/mach.h>     // For macOS memory info
//     #include <pthread.h>       // For thread naming
// #elif defined(__linux__)
//     #include <sys/sysinfo.h>   // For Linux memory info
//     #include <pthread.h>       // For thread naming
// #endif
//
// namespace engine::resources {
//
//     /**
//      * @brief Constructor initializes ResourceManager with given configuration
//      * @param config Configuration parameters for memory limits, async loading, etc.
//      */
//     ResourceManager::ResourceManager(const ResourceManagerConfig& config)
//         : config_(config)                           // Store configuration settings
//         , shouldStop_(false)                        // Initialize background thread control
//         , totalMemoryUsage_(0)                      // Start with zero memory usage
//         , resourceCount_(0)                         // Start with zero resources loaded
//         , nextResourceId_(1)                        // Start resource IDs at 1 (0 is reserved for INVALID_RESOURCE_ID)
//     {
//         // Reserve space in hash maps to avoid frequent rehashing during gameplay
//         // Using load factor of 0.75, so reserve 25% more than expected max resources
//         const std::size_t reserveSize = static_cast<std::size_t>(config_.maxResources * 1.25f);
//         resourcesByPath_.reserve(reserveSize);
//         resourcesById_.reserve(reserveSize);
//
//         // Start background loading thread if async loading is enabled
//         if (config_.enableAsyncLoading) {
//             // Lambda capture ensures proper initialization order
//             backgroundThread_ = std::thread([this]() { backgroundLoadingThread(); });
//
//             // Set thread name for easier debugging (platform-specific)
//             #ifdef _WIN32
//                 // Windows thread naming
//                 const DWORD MS_VC_EXCEPTION = 0x406D1388;
//                 struct THREADNAME_INFO {
//                     DWORD dwType;     // Must be 0x1000
//                     LPCSTR szName;    // Pointer to name (in user addr space)
//                     DWORD dwThreadID; // Thread ID (-1=caller thread)
//                     DWORD dwFlags;    // Reserved for future use, must be zero
//                 } info = { 0x1000, "ResourceManager::BackgroundLoader", GetCurrentThreadId(), 0 };
//
//                 __try {
//                     RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
//                 }
//                 __except (EXCEPTION_EXECUTE_HANDLER) {
//                     // Ignore exception - thread naming is not critical
//                 }
//             #elif defined(__APPLE__) || defined(__linux__)
//                 // POSIX thread naming
//                 pthread_setname_np("ResManager");
//             #endif
//         }
//     }
//
//     /**
//      * @brief Destructor ensures clean shutdown and resource cleanup
//      */
//     ResourceManager::~ResourceManager() {
//         // Signal background thread to stop
//         shouldStop_.store(true, std::memory_order_release);
//
//         // Wait for background thread to finish if it was started
//         if (backgroundThread_.joinable()) {
//             backgroundThread_.join();
//         }
//
//         // Clear all resources - this will trigger RAII cleanup
//         clear();
//     }
//
//     /**
//      * @brief Force reload a resource from disk
//      * @param path File path of resource to reload
//      * @return True if reload was successful
//      */
//     bool ResourceManager::reloadResource(const std::string& path) {
//         std::string normalizedPath = std::filesystem::path(path).make_preferred().string();
//
//         std::unique_lock<std::shared_mutex> lock(resourceMutex_);
//
//         auto it = resourcesByPath_.find(normalizedPath);
//         if (it == resourcesByPath_.end()) {
//             // Resource not loaded - nothing to reload
//             return false;
//         }
//
//         // Call reload on the resource itself
//         bool success = it->second->resource->reload();
//
//         if (success) {
//             // Update load time
//             it->second->loadTime = std::chrono::steady_clock::now();
//         }
//
//         return success;
//     }
//
//     /**
//      * @brief Unload a specific resource from memory
//      * @param path File path of resource to unload
//      * @return True if resource was unloaded
//      */
//     bool ResourceManager::unloadResource(const std::string& path) {
//         std::string normalizedPath = std::filesystem::path(path).make_preferred().string();
//
//         std::unique_lock<std::shared_mutex> lock(resourceMutex_);
//
//         auto it = resourcesByPath_.find(normalizedPath);
//         if (it == resourcesByPath_.end()) {
//             // Resource not loaded
//             return false;
//         }
//
//         // Check if resource is still being referenced
//         if (it->second->resource->isReferenced()) {
//             // Cannot unload resource that is still in use
//             return false;
//         }
//
//         // Update memory usage statistics
//         totalMemoryUsage_.fetch_sub(it->second->resource->getMemorySize(), std::memory_order_acq_rel);
//         resourceCount_.fetch_sub(1, std::memory_order_acq_rel);
//
//         // Remove from both maps
//         ResourceID resourceId = it->second->id;
//         resourcesByPath_.erase(it);
//         resourcesById_.erase(resourceId);
//
//         return true;
//     }
//
//     /**
//      * @brief Clear all loaded resources from memory
//      */
//     void ResourceManager::clear() {
//         std::unique_lock<std::shared_mutex> lock(resourceMutex_);
//
//         // Clear all resources - RAII will handle cleanup
//         resourcesByPath_.clear();
//         resourcesById_.clear();
//
//         // Reset statistics
//         totalMemoryUsage_.store(0, std::memory_order_release);
//         resourceCount_.store(0, std::memory_order_release);
//     }
//
//     /**
//      * @brief Force garbage collection of unused resources
//      * @return Number of resources that were freed
//      */
//     std::size_t ResourceManager::garbageCollect() {
//         std::unique_lock<std::shared_mutex> lock(resourceMutex_);
//
//         std::size_t freedCount = 0;
//         auto currentTime = std::chrono::steady_clock::now();
//
//         // Collect resources to remove (cannot modify container while iterating)
//         std::vector<std::string> resourcesToRemove;
//
//         for (const auto& [path, entry] : resourcesByPath_) {
//             // Skip resources that are still referenced
//             if (entry->resource->isReferenced()) {
//                 continue;
//             }
//
//             // Skip critical priority resources
//             if (entry->priority == LoadPriority::CRITICAL) {
//                 continue;
//             }
//
//             // Check if resource has exceeded cache timeout
//             auto timeSinceLastAccess = currentTime - entry->resource->getLastAccessed();
//             if (timeSinceLastAccess > config_.cacheTimeout) {
//                 resourcesToRemove.push_back(path);
//             }
//         }
//
//         // Remove collected resources
//         for (const auto& path : resourcesToRemove) {
//             auto it = resourcesByPath_.find(path);
//             if (it != resourcesByPath_.end()) {
//                 // Update memory usage statistics
//                 totalMemoryUsage_.fetch_sub(it->second->resource->getMemorySize(), std::memory_order_acq_rel);
//                 resourceCount_.fetch_sub(1, std::memory_order_acq_rel);
//
//                 // Remove from both maps
//                 ResourceID resourceId = it->second->id;
//                 resourcesByPath_.erase(it);
//                 resourcesById_.erase(resourceId);
//
//                 ++freedCount;
//             }
//         }
//
//         return freedCount;
//     }
//
//     /**
//      * @brief Get current memory usage statistics
//      * @return Total memory used by all loaded resources in bytes
//      */
//     std::size_t ResourceManager::getMemoryUsage() const {
//         return totalMemoryUsage_.load(std::memory_order_acquire);
//     }
//
//     /**
//      * @brief Get number of currently loaded resources
//      * @return Count of loaded resources
//      */
//     std::size_t ResourceManager::getResourceCount() const {
//         return resourceCount_.load(std::memory_order_acquire);
//     }
//
//     /**
//      * @brief Check if a resource file exists at the given path
//      * @param path File path to check
//      * @return True if resource exists and can be loaded
//      */
//     bool ResourceManager::resourceExists(const std::string& path) const {
//         // Check if file exists on disk
//         std::filesystem::path filePath(path);
//         if (!std::filesystem::exists(filePath)) {
//             return false;
//         }
//
//         // Check if we have a factory that can handle this file type
//         std::string extension = getFileExtension(path);
//         std::lock_guard<std::mutex> lock(factoryMutex_);
//
//         // Convert to lowercase for case-insensitive comparison
//         std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
//
//         return factories_.find(extension) != factories_.end();
//     }
//
//     /**
//      * @brief Update the resource manager (call once per frame)
//      *
//      * Handles background loading completion, memory management,
//      * and periodic garbage collection.
//      */
//     void ResourceManager::update() {
//         // Process completed background loading tasks
//         // (This is handled automatically by the background thread)
//
//         // Periodic memory management
//         static auto lastGCTime = std::chrono::steady_clock::now();
//         auto currentTime = std::chrono::steady_clock::now();
//
//         // Run garbage collection every 30 seconds
//         if (currentTime - lastGCTime > std::chrono::seconds(30)) {
//             garbageCollect();
//             lastGCTime = currentTime;
//         }
//
//         // Check if we're over memory limits
//         checkMemoryLimits();
//
//         // Optional: Log memory usage statistics in debug builds
//         #ifdef _DEBUG
//         if (config_.enableMemoryProfiling) {
//             static auto lastLogTime = std::chrono::steady_clock::now();
//             if (currentTime - lastLogTime > std::chrono::seconds(10)) {
//                 std::size_t memUsage = getMemoryUsage();
//                 std::size_t resCount = getResourceCount();
//
//                 // Log to debug output (replace with your logging system)
//                 std::cout << "ResourceManager: " << resCount << " resources, "
//                          << (memUsage / (1024.0 * 1024.0)) << " MB used\n";
//
//                 lastLogTime = currentTime;
//             }
//         }
//         #endif
//     }
//
//     /**
//      * @brief Generate unique resource ID from file path using hash function
//      * @param path Normalized file path
//      * @return Unique resource identifier
//      */
//     ResourceID ResourceManager::generateResourceId(const std::string& path) const {
//         // Use std::hash for consistent ID generation
//         // This ensures same path always gets same ID across runs
//         std::hash<std::string> hasher;
//         ResourceID id = hasher(path);
//
//         // Ensure ID is never 0 (reserved for INVALID_RESOURCE_ID)
//         return (id == 0) ? 1 : id;
//     }
//
//     /**
//      * @brief Extract file extension from path
//      * @param path File path
//      * @return File extension including dot (e.g., ".png")
//      */
//     std::string ResourceManager::getFileExtension(const std::string& path) const {
//         std::filesystem::path filePath(path);
//         return filePath.extension().string();
//     }
//
//     /**
//      * @brief Check if memory usage exceeds limits and trigger eviction if needed
//      */
//     void ResourceManager::checkMemoryLimits() {
//         std::size_t currentMemory = totalMemoryUsage_.load(std::memory_order_acquire);
//         std::size_t currentCount = resourceCount_.load(std::memory_order_acquire);
//
//         // Check if we exceed memory or resource count limits
//         bool exceedsMemory = currentMemory > config_.maxMemoryUsage;
//         bool exceedsCount = currentCount > config_.maxResources;
//
//         if (exceedsMemory || exceedsCount) {
//             // Calculate target memory (90% of limit to provide some headroom)
//             std::size_t targetMemory = static_cast<std::size_t>(config_.maxMemoryUsage * 0.9);
//             evictLeastRecentlyUsed(targetMemory);
//         }
//     }
//
//     /**
//      * @brief Evict least recently used resources to free memory
//      * @param targetMemory Target memory usage after eviction
//      */
//     void ResourceManager::evictLeastRecentlyUsed(std::size_t targetMemory) {
//         std::unique_lock<std::shared_mutex> lock(resourceMutex_);
//
//         // Collect all evictable resources with their last access times
//         struct EvictionCandidate {
//             std::string path;
//             std::chrono::steady_clock::time_point lastAccessed;
//             std::size_t memorySize;
//             LoadPriority priority;
//         };
//
//         std::vector<EvictionCandidate> candidates;
//
//         for (const auto& [path, entry] : resourcesByPath_) {
//             // Skip resources that are still referenced
//             if (entry->resource->isReferenced()) {
//                 continue;
//             }
//
//             // Skip critical priority resources
//             if (entry->priority == LoadPriority::CRITICAL) {
//                 continue;
//             }
//
//             candidates.push_back({
//                 path,
//                 entry->resource->getLastAccessed(),
//                 entry->resource->getMemorySize(),
//                 entry->priority
//             });
//         }
//
//         // Sort candidates by priority (low priority first) then by last access time (oldest first)
//         std::sort(candidates.begin(), candidates.end(),
//                  [](const EvictionCandidate& a, const EvictionCandidate& b) {
//                      if (a.priority != b.priority) {
//                          return a.priority < b.priority;  // Lower priority evicted first
//                      }
//                      return a.lastAccessed < b.lastAccessed;  // Older access evicted first
//                  });
//
//         // Evict resources until we reach target memory usage
//         std::size_t currentMemory = totalMemoryUsage_.load(std::memory_order_acquire);
//
//         for (const auto& candidate : candidates) {
//             if (currentMemory <= targetMemory) {
//                 break;  // Reached target memory usage
//             }
//
//             // Find and remove the resource
//             auto it = resourcesByPath_.find(candidate.path);
//             if (it != resourcesByPath_.end()) {
//                 // Update statistics
//                 totalMemoryUsage_.fetch_sub(candidate.memorySize, std::memory_order_acq_rel);
//                 resourceCount_.fetch_sub(1, std::memory_order_acq_rel);
//                 currentMemory -= candidate.memorySize;
//
//                 // Remove from both maps
//                 ResourceID resourceId = it->second->id;
//                 resourcesByPath_.erase(it);
//                 resourcesById_.erase(resourceId);
//             }
//         }
//     }
//
//     /**
//      * @brief Background thread function for asynchronous resource loading
//      */
//     void ResourceManager::backgroundLoadingThread() {
//         while (!shouldStop_.load(std::memory_order_acquire)) {
//             std::vector<std::function<void()>> tasksToProcess;
//
//             // Get all pending tasks from the queue
//             {
//                 std::lock_guard<std::mutex> lock(backgroundMutex_);
//                 if (!loadingQueue_.empty()) {
//                     tasksToProcess = std::move(loadingQueue_);
//                     loadingQueue_.clear();
//                 }
//             }
//
//             // Process all tasks outside of the lock
//             for (auto& task : tasksToProcess) {
//                 if (shouldStop_.load(std::memory_order_acquire)) {
//                     break;  // Stop processing if shutdown requested
//                 }
//
//                 try {
//                     task();  // Execute loading task
//                 } catch (const std::exception& e) {
//                     // Log error and continue (replace with your logging system)
//                     std::cerr << "ResourceManager: Background loading error: " << e.what() << std::endl;
//                 } catch (...) {
//                     // Log unknown error and continue
//                     std::cerr << "ResourceManager: Unknown background loading error" << std::endl;
//                 }
//             }
//
//             // Sleep briefly to avoid busy waiting
//             std::this_thread::sleep_for(std::chrono::milliseconds(16));  // ~60 FPS
//         }
//     }
//
// } // namespace engine::resources