/**
 * @file InputMemoryPools.h
 * @brief Memory pool management for the input system
 * @author Andrés Guerrero
 * @date 13-09-2025
 *
 * Provides centralized memory pool allocation for input events, snapshots,
 * and other frequently allocated objects. Zero-allocation in hot paths.
 */

#pragma once

#include "../core/InputEvent.h"
#include "../core/InputSnapshot.h"
#include "../core/InputTypes.h"

#include "../../memory/allocators/PoolAllocator.h"

#include <memory>
#include <atomic>
#include <mutex>

namespace engine::input {
    /**
     * @brief Memory pool configuration for input system
     */
    struct InputMemoryPoolConfig {
        // Pool sizes
        std::size_t eventPoolSize = 1024; // Input events
        std::size_t snapshotPoolSize = 128; // Input snapshots
        std::size_t bindingPoolSize = 256; // Input bindings
        std::size_t actionStatePoolSize = 512; // Action states
        std::size_t deviceStatePoolSize = 64; // Device states

        // Buffer pools
        std::size_t smallBufferPoolSize = 512; // 256 bytes each
        std::size_t mediumBufferPoolSize = 128; // 1KB each
        std::size_t largeBufferPoolSize = 32; // 4KB each

        // String pools for context names, action names, etc.
        std::size_t stringPoolSize = 256; // 64 bytes each

        // Preallocate all pools at startup
        bool preallocateAll = true;

        // Allow pools to grow dynamically
        bool allowDynamicGrowth = false;

        // Track allocations in debug mode
        bool enableDebugTracking = true;
    };

    /**
     * @brief Centralized memory pool manager for input system
     *
     * Manages all memory pools used by the input system to ensure
     * zero heap allocations during gameplay.
     */
    class InputMemoryPools {
    public:
        /**
         * @brief Constructor
         */
        explicit InputMemoryPools() noexcept;

        /**
         * @brief Destructor
         */
        ~InputMemoryPools();

        // Disable copy, enable move
        InputMemoryPools(const InputMemoryPools&) = delete;
        InputMemoryPools& operator=(const InputMemoryPools&) = delete;
        InputMemoryPools(InputMemoryPools&&) noexcept = default;
        InputMemoryPools& operator=(InputMemoryPools&&) noexcept = default;

        // ============================================================================
        // Initialization
        // ============================================================================

        /**
         * @brief Initialize memory pools
         * @param config Pool configuration
         * @return True if successful
         */
        bool initialize(const InputMemoryPoolConfig& config = {});

        /**
         * @brief Shutdown and release all pools
         */
        void shutdown();

        /**
         * @brief Check if initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            return initialized_;
        }

        // ============================================================================
        // Event Allocation
        // ============================================================================

        /**
         * @brief Allocate input event from pool
         * @return Event pointer or nullptr if pool exhausted
         */
        [[nodiscard]] InputEvent* allocateEvent() noexcept;

        /**
         * @brief Deallocate event back to pool
         * @param event Event to deallocate
         */
        void deallocateEvent(InputEvent* event) noexcept;

        /**
         * @brief Create and initialize event
         * @tparam T Event data type
         * @param type Event type
         * @param deviceId Device ID
         * @param data Event data
         * @return Event pointer or nullptr
         */
        template <typename T>
        [[nodiscard]] InputEvent* createEvent(const InputEventType type,
                                              const DeviceID deviceId,
                                              const T& data) noexcept {
            InputEvent* event = allocateEvent();
            if (!event) return nullptr;

            event->type = type;
            event->deviceId = deviceId;
            event->data = data;
            event->timestamp = InputTimestamp::clock::now();
            event->consumed = false;
            event->synthetic = false;

            return event;
        }

        // ============================================================================
        // Snapshot Allocation
        // ============================================================================

        /**
         * @brief Allocate input snapshot from pool
         * @return Snapshot pointer or nullptr if pool exhausted
         */
        [[nodiscard]] InputSnapshot* allocateSnapshot() noexcept;

        /**
         * @brief Deallocate snapshot back to pool
         * @param snapshot Snapshot to deallocate
         */
        void deallocateSnapshot(InputSnapshot* snapshot) noexcept;

        // ============================================================================
        // Buffer Allocation
        // ============================================================================

        enum class BufferSize : std::uint8_t {
            SMALL = 0, // 256 bytes
            MEDIUM = 1, // 1KB
            LARGE = 2 // 4KB
        };

        /**
         * @brief Allocate buffer from appropriate pool
         * @param size Required size in bytes
         * @return Buffer pointer or nullptr
         */
        [[nodiscard]] void* allocateBuffer(std::size_t size) noexcept;

        /**
         * @brief Deallocate buffer back to pool
         * @param buffer Buffer to deallocate
         * @param size Buffer size
         */
        void deallocateBuffer(void* buffer, std::size_t size) noexcept;

        /**
         * @brief Get buffer of specific size category
         * @param size Size category
         * @return Buffer pointer or nullptr
         */
        [[nodiscard]] void* getBuffer(BufferSize size) noexcept;

        /**
         * @brief Return buffer to pool
         * @param buffer Buffer to return
         * @param size Size category
         */
        void returnBuffer(void* buffer, BufferSize size) noexcept;

        // ============================================================================
        // String Allocation
        // ============================================================================

        /**
         * @brief String buffer for temporary strings
         */
        struct alignas(void*) StringBuffer {
            char data[64];
            std::atomic<bool> inUse{false};

            StringBuffer() noexcept {
                data[0] = '\0';
            }
        };

        /**
         * @brief Allocate string buffer from pool
         * @return String buffer or nullptr
         */
        [[nodiscard]] StringBuffer* allocateString() const noexcept;

        /**
         * @brief Deallocate string buffer back to pool
         * @param buffer String buffer to deallocate
         */
        void deallocateString(StringBuffer* buffer) const noexcept;

        // ============================================================================
        // Statistics
        // ============================================================================

        /**
         * @brief Pool statistics
         */
        struct Statistics {
            // Event pool
            std::atomic<std::uint32_t> eventsAllocated{0};
            std::atomic<std::uint32_t> eventsInUse{0};
            std::atomic<std::uint32_t> eventPoolExhausted{0};

            // Snapshot pool
            std::atomic<std::uint32_t> snapshotsAllocated{0};
            std::atomic<std::uint32_t> snapshotsInUse{0};
            std::atomic<std::uint32_t> snapshotPoolExhausted{0};

            // Buffer pools
            std::atomic<std::uint32_t> buffersAllocated{0};
            std::atomic<std::uint32_t> buffersInUse{0};
            std::atomic<std::uint32_t> bufferPoolExhausted{0};

            // String pool
            std::atomic<std::uint32_t> stringsAllocated{0};
            std::atomic<std::uint32_t> stringsInUse{0};
            std::atomic<std::uint32_t> stringPoolExhausted{0};

            // Memory usage
            std::atomic<std::size_t> totalMemoryAllocated{0};
            std::atomic<std::size_t> totalMemoryInUse{0};
            std::atomic<std::size_t> peakMemoryUsage{0};

            void reset() noexcept {
                eventsAllocated = 0;
                eventsInUse = 0;
                eventPoolExhausted = 0;
                snapshotsAllocated = 0;
                snapshotsInUse = 0;
                snapshotPoolExhausted = 0;
                buffersAllocated = 0;
                buffersInUse = 0;
                bufferPoolExhausted = 0;
                stringsAllocated = 0;
                stringsInUse = 0;
                stringPoolExhausted = 0;
                totalMemoryAllocated = 0;
                totalMemoryInUse = 0;
                peakMemoryUsage = 0;
            }
        };

        /**
         * @brief Get pool statistics
         */
        [[nodiscard]] const Statistics& getStatistics() const noexcept {
            return stats_;
        }

        /**
         * @brief Reset statistics
         */
        void resetStatistics() const noexcept {
            stats_.reset();
        }

        // ============================================================================
        // Pool Management
        // ============================================================================

        /**
         * @brief Reset all pools (deallocate all objects)
         */
        void resetAllPools() noexcept;

        /**
         * @brief Defragment pools (if applicable)
         * @return Number of objects moved
         */
        std::size_t defragmentPools();

        /**
         * @brief Get total memory usage
         */
        [[nodiscard]] std::size_t getTotalMemoryUsage() const noexcept;

        /**
         * @brief Get available events in pool
         */
        [[nodiscard]] std::size_t getAvailableEvents() const noexcept;

        /**
         * @brief Get available snapshots in pool
         */
        [[nodiscard]] std::size_t getAvailableSnapshots() const noexcept;

        /**
         * @brief Validate pool integrity
         * @return True if all pools are valid
         */
        [[nodiscard]] bool validatePools() const noexcept;

    private:
        // ============================================================================
        // Member Variables
        // ============================================================================

        // Configuration
        InputMemoryPoolConfig config_;
        bool initialized_ = false;

        // Memory management
        // memory::MemoryManager* memoryManager_;
        // bool ownsMemoryManager_ = false;

        // Event pool
        std::unique_ptr<memory::PoolAllocator> eventPool_;

        // Snapshot pool
        std::unique_ptr<memory::PoolAllocator> snapshotPool_;

        // Buffer pools (different sizes)
        std::unique_ptr<memory::PoolAllocator> smallBufferPool_;
        std::unique_ptr<memory::PoolAllocator> mediumBufferPool_;
        std::unique_ptr<memory::PoolAllocator> largeBufferPool_;

        // String pool
        std::unique_ptr<memory::PoolAllocator> stringPool_;

        // Statistics
        mutable Statistics stats_;

        // Thread safety
        mutable std::mutex poolMutex_;

        // Debug tracking
#ifdef _DEBUG
        struct AllocationInfo {
            void* ptr;
            std::size_t size;
            std::chrono::steady_clock::time_point timestamp;
            std::string file;
            int line;
        };

        std::unordered_map<void*, AllocationInfo> allocations_;
        mutable std::mutex debugMutex_;
#endif

        // ============================================================================
        // Internal Methods
        // ============================================================================

        /**
         * @brief Create all memory pools
         * @return True if successful
         */
        bool createPools();

        /**
         * @brief Destroy all memory pools
         */
        void destroyPools();

        /**
         * @brief Update memory statistics
         */
        void updateMemoryStats() const;

        /**
         * @brief Get buffer size for allocation
         * @param size Requested size
         * @return Buffer size category
         */
        [[nodiscard]] static BufferSize getBufferSizeCategory(std::size_t size) noexcept;

        /**
         * @brief Track allocation in debug mode
         * @param ptr Allocated pointer
         * @param size Allocation size
         * @param file Source file
         * @param line Source line
         */
        void trackAllocation(void* ptr, std::size_t size,
                             const char* file = "", int line = 0);

        /**
         * @brief Untrack allocation in debug mode
         * @param ptr Pointer to untrack
         */
        void untrackAllocation(void* ptr);
    };
} // namespace engine::input
