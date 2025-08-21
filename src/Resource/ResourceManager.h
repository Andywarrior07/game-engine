// //
// // Created by Andres Guerrero on 05-08-25.
// //
//
// #pragma once
//
// #include <memory>           // For smart pointers and memory management
// #include <unordered_map>    // For fast resource lookup by ID
// #include <string>           // For resource identifiers and paths
// #include <mutex>            // For thread-safe operations
// #include <shared_mutex>     // For reader-writer locks (C++17)
// #include <atomic>           // For lock-free reference counting
// #include <functional>       // For std::function callbacks
// #include <type_traits>      // For template metaprogramming
// #include <optional>         // For optional return values (C++17)
// #include <vector>           // For resource containers
// #include <chrono>           // For timestamp tracking
// #include <thread>           // For background loading thread support
// #include <filesystem>       // For file system operations
// #include <algorithm>        // For std::transform
//
//
// namespace engine::resources {
//
//     // Forward declarations for circular dependency resolution
//     template<typename T>
//     class ResourceHandle;
//
//     class IResource;
//     class ResourceManager;
//
//     class MemoryManager;
//
//     // Unique resource identifier type - strongly typed to prevent mixing with other IDs
//     using ResourceID = std::uint64_t;
//
//     // Invalid resource ID constant for error checking
//     constexpr ResourceID INVALID_RESOURCE_ID = 0;
//
//     // Resource loading priority levels for memory management
//     enum class LoadPriority : std::uint8_t {
//         LOW = 0,        // Background loading, can be evicted easily
//         NORMAL = 1,     // Standard loading priority
//         HIGH = 2,       // Important resources, keep in memory longer
//         CRITICAL = 3    // Never evict, keep always loaded
//     };
//
//     // Resource loading states for async loading system
//     enum class ResourceState : std::uint8_t {
//         UNLOADED = 0,   // Resource not loaded into memory
//         LOADING = 1,    // Currently being loaded asynchronously
//         LOADED = 2,     // Successfully loaded and ready to use
//         FAILED = 3      // Loading failed, contains error information
//     };
//
//     /**
//      * @brief Base interface for all resources managed by the ResourceManager
//      *
//      * This interface provides the common functionality that all resources must implement.
//      * Uses virtual inheritance to avoid diamond problem in resource hierarchies.
//      */
//     class IResource {
//     public:
//         // Virtual destructor ensures proper cleanup in derived classes
//         virtual ~IResource() = default;
//
//         // Get the unique identifier for this resource
//         virtual ResourceID getId() const noexcept = 0;
//
//         // Get the file path this resource was loaded from
//         virtual const std::string& getPath() const noexcept = 0;
//
//         // Get the current loading state of this resource
//         virtual ResourceState getState() const noexcept = 0;
//
//         // Get the memory size used by this resource in bytes
//         virtual std::size_t getMemorySize() const noexcept = 0;
//
//         // Get the priority level of this resource
//         virtual LoadPriority getPriority() const noexcept = 0;
//
//         // Get the last access timestamp for LRU cache management
//         virtual std::chrono::steady_clock::time_point getLastAccessed() const noexcept = 0;
//
//         // Force reload of this resource from disk
//         virtual bool reload() = 0;
//
//         // Check if this resource is currently being referenced
//         virtual bool isReferenced() const noexcept = 0;
//
//         // Set the current loading state (added this method)
//         virtual void setState(ResourceState newState) noexcept = 0;
//
//     protected:
//         // Protected constructor - only derived classes can instantiate
//         IResource() = default;
//
//         // Non-copyable but moveable for performance
//         IResource(const IResource&) = delete;
//         IResource& operator=(const IResource&) = delete;
//         IResource(IResource&&) = default;
//         IResource& operator=(IResource&&) = default;
//     };
//
//     /**
//      * @brief Template base class for typed resources
//      *
//      * Provides type-safe resource implementation with automatic lifetime management.
//      * Uses CRTP (Curiously Recurring Template Pattern) for static polymorphism.
//      */
//     template<typename Derived>
//     class Resource : public IResource {
//     public:
//         // Type alias for the derived resource type
//         using ResourceType = Derived;
//
//         /**
//          * @brief Constructor for Resource base class
//          * @param id Unique identifier for this resource
//          * @param path File path where resource is stored
//          * @param priority Loading priority for memory management
//          */
//         explicit Resource(ResourceID id, std::string path, LoadPriority priority = LoadPriority::NORMAL)
//             : id_(id)
//             , path_(std::move(path))
//             , priority_(priority)
//             , state_(ResourceState::UNLOADED)
//             , lastAccessed_(std::chrono::steady_clock::now())
//             , referenceCount_(0)
//         {
//             // Empty constructor body - all initialization in member initializer list
//         }
//
//         // Explicitly deleted copy constructor and assignment
//         Resource(const Resource&) = delete;
//         Resource& operator=(const Resource&) = delete;
//
//         // Move constructor
//         Resource(Resource&& other) noexcept
//             : IResource(std::move(other))
//             , id_(other.id_)
//             , path_(std::move(const_cast<std::string&>(other.path_)))
//             , priority_(other.priority_)
//             , state_(other.state_.load())
//             , lastAccessed_(other.lastAccessed_.load())
//             , referenceCount_(other.referenceCount_.load())
//         {
//         }
//
//         // Move assignment operator
//         Resource& operator=(Resource&& other) noexcept {
//             if (this != &other) {
//                 IResource::operator=(std::move(other));
//                 // Note: const members (id_, path_, priority_) cannot be moved after construction
//                 // This is by design - resources should not change their identity after creation
//                 state_.store(other.state_.load());
//                 lastAccessed_.store(other.lastAccessed_.load());
//                 referenceCount_.store(other.referenceCount_.load());
//             }
//             return *this;
//         }
//
//         // Implement IResource interface
//         ResourceID getId() const noexcept override {
//             return id_;
//         }
//
//         const std::string& getPath() const noexcept override {
//             return path_;
//         }
//
//         ResourceState getState() const noexcept override {
//             return state_.load(std::memory_order_acquire);
//         }
//
//         LoadPriority getPriority() const noexcept override {
//             return priority_;
//         }
//
//         std::chrono::steady_clock::time_point getLastAccessed() const noexcept override {
//             return lastAccessed_.load(std::memory_order_acquire);
//         }
//
//         bool isReferenced() const noexcept override {
//             return referenceCount_.load(std::memory_order_acquire) > 0;
//         }
//
//         // Update last accessed timestamp for LRU management
//         void updateLastAccessed() const noexcept {
//             lastAccessed_.store(std::chrono::steady_clock::now(), std::memory_order_release);
//         }
//
//     protected:
//         // Set the current loading state (thread-safe)
//         void setState(ResourceState newState) noexcept {
//             state_.store(newState, std::memory_order_release);
//         }
//
//         // Increment reference count when handle is created
//         void addReference() const noexcept {
//             referenceCount_.fetch_add(1, std::memory_order_acq_rel);
//         }
//
//         // Decrement reference count when handle is destroyed
//         void removeReference() const noexcept {
//             referenceCount_.fetch_sub(1, std::memory_order_acq_rel);
//         }
//
//     private:
//         const ResourceID id_;                                           // Immutable resource identifier
//         const std::string path_;                                        // Immutable file path
//         const LoadPriority priority_;                                   // Immutable loading priority
//
//         std::atomic<ResourceState> state_;                              // Current loading state (thread-safe)
//         mutable std::atomic<std::chrono::steady_clock::time_point> lastAccessed_;  // Last access time for LRU
//         mutable std::atomic<std::uint32_t> referenceCount_;            // Number of active handles (thread-safe)
//
//         // Friend class to allow ResourceHandle access to reference counting
//         template<typename T>
//         friend class ResourceHandle;
//     };
//
//     /**
//      * @brief Smart handle for resources with automatic lifetime management
//      *
//      * Similar to std::shared_ptr but optimized for resource management.
//      * Provides type-safe access to resources with automatic reference counting.
//      */
//     template<typename T>
//     class ResourceHandle {
//     public:
//         static_assert(std::is_base_of_v<IResource, T>, "T must derive from IResource");
//
//         // Default constructor creates invalid handle
//         ResourceHandle() noexcept : resource_(nullptr) {}
//
//         // Constructor from raw resource pointer
//         explicit ResourceHandle(T* resource) noexcept : resource_(resource) {
//             if (resource_) {
//                 resource_->addReference();
//                 resource_->updateLastAccessed();
//             }
//         }
//
//         // Copy constructor increments reference count
//         ResourceHandle(const ResourceHandle& other) noexcept : resource_(other.resource_) {
//             if (resource_) {
//                 resource_->addReference();
//                 resource_->updateLastAccessed();
//             }
//         }
//
//         // Move constructor transfers ownership
//         ResourceHandle(ResourceHandle&& other) noexcept : resource_(other.resource_) {
//             other.resource_ = nullptr;
//         }
//
//         // Copy assignment operator
//         ResourceHandle& operator=(const ResourceHandle& other) noexcept {
//             if (this != &other) {
//                 // Release current resource
//                 if (resource_) {
//                     resource_->removeReference();
//                 }
//
//                 // Acquire new resource
//                 resource_ = other.resource_;
//                 if (resource_) {
//                     resource_->addReference();
//                     resource_->updateLastAccessed();
//                 }
//             }
//             return *this;
//         }
//
//         // Move assignment operator
//         ResourceHandle& operator=(ResourceHandle&& other) noexcept {
//             if (this != &other) {
//                 // Release current resource
//                 if (resource_) {
//                     resource_->removeReference();
//                 }
//
//                 // Transfer ownership
//                 resource_ = other.resource_;
//                 other.resource_ = nullptr;
//             }
//             return *this;
//         }
//
//         // Destructor decrements reference count
//         ~ResourceHandle() {
//             if (resource_) {
//                 resource_->removeReference();
//             }
//         }
//
//         // Dereference operators for direct resource access
//         T& operator*() const noexcept {
//             resource_->updateLastAccessed();
//             return *resource_;
//         }
//
//         T* operator->() const noexcept {
//             resource_->updateLastAccessed();
//             return resource_;
//         }
//
//         // Get raw pointer to resource
//         T* get() const noexcept {
//             if (resource_) {
//                 resource_->updateLastAccessed();
//             }
//             return resource_;
//         }
//
//         // Check if handle is valid
//         bool isValid() const noexcept {
//             return resource_ != nullptr;
//         }
//
//         // Check if resource is loaded and ready to use
//         bool isReady() const noexcept {
//             return resource_ && resource_->getState() == ResourceState::LOADED;
//         }
//
//         // Explicit bool conversion for if statements
//         explicit operator bool() const noexcept {
//             return isValid();
//         }
//
//         // Reset handle to invalid state
//         void reset() noexcept {
//             if (resource_) {
//                 resource_->removeReference();
//                 resource_ = nullptr;
//             }
//         }
//
//     private:
//         T* resource_;  // Raw pointer to resource (managed by ResourceManager)
//     };
//
//     /**
//      * @brief Factory interface for creating resources of specific types
//      *
//      * Each resource type should have a corresponding factory that knows how to
//      * load that resource from disk or other sources.
//      */
//     template<typename T>
//     class IResourceFactory {
//     public:
//         static_assert(std::is_base_of_v<IResource, T>, "T must derive from IResource");
//
//         virtual ~IResourceFactory() = default;
//
//         /**
//          * @brief Create and load a resource from the given path
//          * @param id Unique identifier for the resource
//          * @param path File path to load from
//          * @param priority Loading priority
//          * @return Unique pointer to created resource, or nullptr on failure
//          */
//         virtual std::unique_ptr<T> createResource(ResourceID id, const std::string& path, LoadPriority priority) = 0;
//
//         /**
//          * @brief Get file extensions supported by this factory
//          * @return Vector of supported file extensions (including dot, e.g., ".png")
//          */
//         virtual std::vector<std::string> getSupportedExtensions() const = 0;
//
//     protected:
//         IResourceFactory() = default;
//     };
//
//     /**
//      * @brief Configuration parameters for ResourceManager
//      */
//     struct ResourceManagerConfig {
//         std::size_t maxMemoryUsage = 512 * 1024 * 1024;    // Maximum memory usage in bytes (512MB default)
//         std::size_t maxResources = 10000;                   // Maximum number of resources to keep in memory
//         bool enableAsyncLoading = true;                     // Enable background loading thread
//         bool enableMemoryProfiling = false;                // Enable detailed memory usage tracking
//         std::chrono::seconds cacheTimeout{300};            // Time before unused resources are evicted (5 minutes)
//     };
//
//     /**
//      * @brief High-performance resource manager with RAII and optimal memory management
//      *
//      * This class manages the lifetime of all game resources, providing:
//      * - Type-safe resource loading and access
//      * - Automatic memory management with LRU eviction
//      * - Thread-safe operations for multithreaded games
//      * - Asynchronous loading for smooth gameplay
//      * - Memory profiling and optimization
//      */
//     class ResourceManager {
//     public:
//         /**
//          * @brief Constructor with configuration
//          * @param config Configuration parameters for the resource manager
//          */
//         explicit ResourceManager(const ResourceManagerConfig& config = {});
//
//         // Destructor ensures all resources are properly cleaned up
//         ~ResourceManager();
//
//         // Non-copyable but moveable
//         ResourceManager(const ResourceManager&) = delete;
//         ResourceManager& operator=(const ResourceManager&) = delete;
//         ResourceManager(ResourceManager&&) = default;
//         ResourceManager& operator=(ResourceManager&&) = default;
//
//         /**
//          * @brief Register a factory for a specific resource type
//          * @tparam T Resource type
//          * @param factory Unique pointer to factory instance
//          */
//         template<typename T>
//         void registerFactory(std::unique_ptr<IResourceFactory<T>> factory) {
//             static_assert(std::is_base_of_v<IResource, T>, "T must derive from IResource");
//
//             // Thread-safe factory registration
//             std::lock_guard<std::mutex> lock(factoryMutex_);
//
//             // Get supported extensions from factory
//             const auto extensions = factory->getSupportedExtensions();
//
//             // Register factory for each supported extension
//             for (const auto& ext : extensions) {
//                 // Convert extension to lowercase for case-insensitive matching
//                 std::string lowerExt = ext;
//                 std::transform(lowerExt.begin(), lowerExt.end(), lowerExt.begin(), ::tolower);
//
//                 // Store factory with proper type information
//                 factories_[lowerExt] = std::make_unique<FactoryWrapper<T>>(std::move(factory));
//             }
//         }
//
//         /**
//          * @brief Load a resource synchronously
//          * @tparam T Resource type to load
//          * @param path File path to resource
//          * @param priority Loading priority (default: NORMAL)
//          * @return Handle to loaded resource, or invalid handle on failure
//          */
//         template<typename T>
//         ResourceHandle<T> loadResource(const std::string& path, LoadPriority priority = LoadPriority::NORMAL) {
//             return loadResourceInternal<T>(path, priority, false);  // false = synchronous loading
//         }
//
//         /**
//          * @brief Load a resource asynchronously
//          * @tparam T Resource type to load
//          * @param path File path to resource
//          * @param callback Optional callback when loading completes
//          * @param priority Loading priority (default: NORMAL)
//          * @return Handle to resource (may not be loaded yet)
//          */
//         template<typename T>
//         ResourceHandle<T> loadResourceAsync(const std::string& path,
//                                            std::function<void(ResourceHandle<T>)> callback = nullptr,
//                                            LoadPriority priority = LoadPriority::NORMAL) {
//             // If async loading is disabled, fall back to synchronous loading
//             if (!config_.enableAsyncLoading) {
//                 auto handle = loadResource<T>(path, priority);
//                 if (callback) {
//                     callback(handle);  // Call callback immediately
//                 }
//                 return handle;
//             }
//
//             return loadResourceInternal<T>(path, priority, true);   // true = asynchronous loading
//         }
//
//         /**
//          * @brief Get an already loaded resource by path
//          * @tparam T Resource type
//          * @param path File path of resource
//          * @return Handle to resource if loaded, invalid handle otherwise
//          */
//         template<typename T>
//         ResourceHandle<T> getResource(const std::string& path) const {
//             // Normalize path for consistent lookup
//             std::string normalizedPath = std::filesystem::path(path).make_preferred().string();
//
//             // Use shared lock for concurrent read access
//             std::shared_lock<std::shared_mutex> lock(resourceMutex_);
//
//             auto it = resourcesByPath_.find(normalizedPath);
//             if (it != resourcesByPath_.end() && it->second->resource->getState() == ResourceState::LOADED) {
//                 // Resource exists and is loaded - return handle
//                 T* resourcePtr = static_cast<T*>(it->second->resource.get());
//                 return ResourceHandle<T>(resourcePtr);
//             }
//
//             // Resource not found or not loaded
//             return ResourceHandle<T>();
//         }
//
//         /**
//          * @brief Get an already loaded resource by ID
//          * @tparam T Resource type
//          * @param id Resource identifier
//          * @return Handle to resource if loaded, invalid handle otherwise
//          */
//         template<typename T>
//         ResourceHandle<T> getResource(ResourceID id) const {
//             // Use shared lock for concurrent read access
//             std::shared_lock<std::shared_mutex> lock(resourceMutex_);
//
//             auto it = resourcesById_.find(id);
//             if (it != resourcesById_.end() && it->second->resource->getState() == ResourceState::LOADED) {
//                 // Resource exists and is loaded - return handle
//                 T* resourcePtr = static_cast<T*>(it->second->resource.get());
//                 return ResourceHandle<T>(resourcePtr);
//             }
//
//             // Resource not found or not loaded
//             return ResourceHandle<T>();
//         }
//
//         /**
//          * @brief Preload resources from a list of paths
//          * @tparam T Resource type
//          * @param paths Vector of file paths to preload
//          * @param priority Loading priority for all resources
//          */
//         template<typename T>
//         void preloadResources(const std::vector<std::string>& paths, LoadPriority priority = LoadPriority::NORMAL) {
//             // Sort paths by priority to load critical resources first
//             std::vector<std::string> sortedPaths = paths;
//
//             // If async loading is enabled, load all resources asynchronously
//             if (config_.enableAsyncLoading) {
//                 for (const auto& path : sortedPaths) {
//                     loadResourceAsync<T>(path, nullptr, priority);
//                 }
//             } else {
//                 // Load synchronously - consider loading in order of priority
//                 for (const auto& path : sortedPaths) {
//                     loadResource<T>(path, priority);
//                 }
//             }
//         }
//
//         /**
//          * @brief Force reload a resource from disk
//          * @param path File path of resource to reload
//          * @return True if reload was successful
//          */
//         bool reloadResource(const std::string& path);
//
//         /**
//          * @brief Unload a specific resource
//          * @param path File path of resource to unload
//          * @return True if resource was unloaded
//          */
//         bool unloadResource(const std::string& path);
//
//         /**
//          * @brief Clear all loaded resources
//          */
//         void clear();
//
//         /**
//          * @brief Force garbage collection of unused resources
//          * @return Number of resources that were freed
//          */
//         std::size_t garbageCollect();
//
//         /**
//          * @brief Get current memory usage statistics
//          * @return Total memory used by all loaded resources in bytes
//          */
//         std::size_t getMemoryUsage() const;
//
//         /**
//          * @brief Get number of currently loaded resources
//          * @return Count of loaded resources
//          */
//         std::size_t getResourceCount() const;
//
//         /**
//          * @brief Check if a resource exists at the given path
//          * @param path File path to check
//          * @return True if resource exists and can be loaded
//          */
//         bool resourceExists(const std::string& path) const;
//
//         /**
//          * @brief Update the resource manager (call once per frame)
//          *
//          * This handles background loading completion, memory management,
//          * and periodic garbage collection.
//          */
//         void update();
//
//     private:
//         // Internal structure for tracking resource metadata
//         struct ResourceEntry {
//             std::unique_ptr<IResource> resource;        // Actual resource instance
//             ResourceID id;                              // Unique identifier
//             std::string path;                          // File path
//             LoadPriority priority;                     // Loading priority
//             std::chrono::steady_clock::time_point loadTime;  // When resource was loaded
//         };
//
//         // Base class for factory storage
//         struct IFactoryBase {
//             virtual ~IFactoryBase() = default;
//         };
//
//         // Template wrapper for type-safe factory storage
//         template<typename T>
//         struct FactoryWrapper : public IFactoryBase {
//             std::unique_ptr<IResourceFactory<T>> factory;
//
//             explicit FactoryWrapper(std::unique_ptr<IResourceFactory<T>> f)
//                 : factory(std::move(f)) {}
//         };
//
//         // Generate unique resource ID from path
//         ResourceID generateResourceId(const std::string& path) const;
//
//         // Get file extension from path
//         std::string getFileExtension(const std::string& path) const;
//
//         // Find factory for given file extension
//         template<typename T>
//         IResourceFactory<T>* findFactory(const std::string& extension) const {
//             std::lock_guard<std::mutex> lock(factoryMutex_);
//
//             // Convert to lowercase for case-insensitive matching
//             std::string lowerExt = extension;
//             std::transform(lowerExt.begin(), lowerExt.end(), lowerExt.begin(), ::tolower);
//
//             auto it = factories_.find(lowerExt);
//             if (it == factories_.end()) {
//                 return nullptr;
//             }
//
//             // Cast back to concrete factory type
//             auto* wrapper = static_cast<FactoryWrapper<T>*>(it->second.get());
//             return wrapper->factory.get();
//         }
//
//         // Internal implementation of resource loading
//         template<typename T>
//         ResourceHandle<T> loadResourceInternal(const std::string& path, LoadPriority priority, bool async) {
//             // Normalize path separators and resolve relative paths
//             std::string normalizedPath = std::filesystem::path(path).make_preferred().string();
//
//             // Generate unique ID for this resource based on normalized path
//             ResourceID resourceId = generateResourceId(normalizedPath);
//
//             // Check if resource is already loaded (reader lock for concurrent access)
//             {
//                 std::shared_lock<std::shared_mutex> lock(resourceMutex_);
//                 auto it = resourcesByPath_.find(normalizedPath);
//                 if (it != resourcesByPath_.end()) {
//                     // Resource already exists - return handle to existing resource
//                     T* resourcePtr = static_cast<T*>(it->second->resource.get());
//                     return ResourceHandle<T>(resourcePtr);
//                 }
//             }
//
//             // Resource not loaded yet - acquire write lock to add it
//             std::unique_lock<std::shared_mutex> lock(resourceMutex_);
//
//             // Double-check pattern - another thread might have loaded it while we waited for write lock
//             auto it = resourcesByPath_.find(normalizedPath);
//             if (it != resourcesByPath_.end()) {
//                 T* resourcePtr = static_cast<T*>(it->second->resource.get());
//                 return ResourceHandle<T>(resourcePtr);
//             }
//
//             // Get file extension to find appropriate factory
//             std::string extension = getFileExtension(normalizedPath);
//             IResourceFactory<T>* factory = findFactory<T>(extension);
//
//             if (!factory) {
//                 // No factory registered for this file type
//                 return ResourceHandle<T>();  // Return invalid handle
//             }
//
//             // Create resource entry with placeholder
//             auto entry = std::make_unique<ResourceEntry>();
//             entry->id = resourceId;
//             entry->path = normalizedPath;
//             entry->priority = priority;
//             entry->loadTime = std::chrono::steady_clock::now();
//
//             if (async) {
//                 // For async loading, create resource in LOADING state
//                 entry->resource = factory->createResource(resourceId, normalizedPath, priority);
//                 if (!entry->resource) {
//                     return ResourceHandle<T>();
//                 }
//                 entry->resource->setState(ResourceState::LOADING);
//
//                 // Store entry in maps
//                 T* resourcePtr = static_cast<T*>(entry->resource.get());
//                 ResourceEntry* entryPtr = entry.get();
//                 resourcesByPath_[normalizedPath] = std::move(entry);
//                 resourcesById_[resourceId] = entryPtr;
//
//                 // Add loading task to background queue
//                 {
//                     std::lock_guard<std::mutex> bgLock(backgroundMutex_);
//                     loadingQueue_.emplace_back([this, factory, resourceId, normalizedPath, priority]() {
//                         // This lambda runs in background thread
//                         auto loadedResource = factory->createResource(resourceId, normalizedPath, priority);
//
//                         if (loadedResource) {
//                             // Loading successful - update resource in place
//                             std::unique_lock<std::shared_mutex> writeLock(resourceMutex_);
//                             auto it = resourcesById_.find(resourceId);
//                             if (it != resourcesById_.end()) {
//                                 it->second->resource = std::move(loadedResource);
//                                 it->second->resource->setState(ResourceState::LOADED);
//
//                                 // Update memory usage statistics
//                                 totalMemoryUsage_.fetch_add(it->second->resource->getMemorySize(), std::memory_order_acq_rel);
//                                 resourceCount_.fetch_add(1, std::memory_order_acq_rel);
//                             }
//                         } else {
//                             // Loading failed - mark resource as failed
//                             std::unique_lock<std::shared_mutex> writeLock(resourceMutex_);
//                             auto it = resourcesById_.find(resourceId);
//                             if (it != resourcesById_.end()) {
//                                 it->second->resource->setState(ResourceState::FAILED);
//                             }
//                         }
//                     });
//                 }
//
//                 // Return handle to resource in LOADING state
//                 return ResourceHandle<T>(resourcePtr);
//
//             } else {
//                 // Synchronous loading - load immediately
//                 entry->resource = factory->createResource(resourceId, normalizedPath, priority);
//
//                 if (!entry->resource) {
//                     // Loading failed
//                     return ResourceHandle<T>();
//                 }
//
//                 // Update memory usage statistics
//                 totalMemoryUsage_.fetch_add(entry->resource->getMemorySize(), std::memory_order_acq_rel);
//                 resourceCount_.fetch_add(1, std::memory_order_acq_rel);
//
//                 // Check if we need to free memory
//                 checkMemoryLimits();
//
//                 // Store entry in maps
//                 T* resourcePtr = static_cast<T*>(entry->resource.get());
//                 ResourceEntry* entryPtr = entry.get();
//                 resourcesByPath_[normalizedPath] = std::move(entry);
//                 resourcesById_[resourceId] = entryPtr;
//
//                 return ResourceHandle<T>(resourcePtr);
//             }
//         }
//
//         // Check if memory usage exceeds limits and trigger eviction
//         void checkMemoryLimits();
//
//         // Evict least recently used resources
//         void evictLeastRecentlyUsed(std::size_t targetMemory);
//
//         // Background thread function for async loading
//         void backgroundLoadingThread();
//
//         // Configuration
//         ResourceManagerConfig config_;
//
//         // Resource storage - uses unordered_map for O(1) lookup by path and ID
//         std::unordered_map<std::string, std::unique_ptr<ResourceEntry>> resourcesByPath_;
//         std::unordered_map<ResourceID, ResourceEntry*> resourcesById_;
//
//         // Factory registry for different resource types
//         std::unordered_map<std::string, std::unique_ptr<IFactoryBase>> factories_;
//
//         // Thread synchronization
//         mutable std::shared_mutex resourceMutex_;       // Reader-writer lock for resource access
//         mutable std::mutex factoryMutex_;               // Mutex for factory registration
//         std::mutex backgroundMutex_;                    // Mutex for background loading queue
//
//         // Background loading
//         std::vector<std::function<void()>> loadingQueue_;  // Queue of loading tasks
//         std::thread backgroundThread_;                     // Background loading thread
//         std::atomic<bool> shouldStop_;                     // Signal to stop background thread
//
//         // Statistics and profiling
//         mutable std::atomic<std::size_t> totalMemoryUsage_;  // Total memory used by resources
//         mutable std::atomic<std::size_t> resourceCount_;     // Number of loaded resources
//
//         // Next available resource ID (atomic for thread safety)
//         std::atomic<ResourceID> nextResourceId_;
//     };
//
// } // namespace engine::resources