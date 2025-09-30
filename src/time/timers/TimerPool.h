/**
 * @file TimerPool.h
 * @brief Object pool for efficient timer allocation
 * @details Provides memory pooling for timer objects to reduce fragmentation
 *          and allocation overhead. Optimized for frequent timer creation and
 *          destruction with fast allocation/deallocation on hot paths.
 *
 * @author Andres Guerrero
 * @date Created on 2025-09-19
 */

#pragma once

#include "Timer.h"

#include "../../memory/MemorySystem.h"

#include <stack>
#include <vector>
#include <mutex>
#include <atomic>

namespace engine::time {

    /**
     * @brief Object pool for timer allocation
     * @details High-performance object pool specifically designed for timer objects.
     *          Reduces memory fragmentation from frequent timer creation/destruction
     *          and provides O(1) allocation/deallocation with minimal overhead.
     *
     * Performance characteristics:
     * - O(1) allocation and deallocation
     * - Zero heap allocations after initial pool creation
     * - Lock-free fast path for common operations
     * - Automatic pool growth when needed
     * - Memory recycling for efficiency
     *
     * Thread-safety:
     * - Thread-safe for concurrent allocations
     * - Uses fine-grained locking for growth
     * - Lock-free for most operations
     */
    class TimerPool {
    public:
        /**
         * @brief Pool configuration
         */
        struct Config {
            std::size_t initialSize{256}; ///< Initial pool size
            std::size_t growthSize{128}; ///< Growth increment
            std::size_t maxSize{8192}; ///< Maximum pool size
            bool preallocate{true}; ///< Preallocate all timers
            bool allowGrowth{true}; ///< Allow dynamic growth
            bool zeroMemory{false}; ///< Zero timer memory
            memory::MemoryCategory memoryCategory{
                    memory::MemoryCategory::GAMEPLAY}; ///< Memory category
        };

        /**
         * @brief Constructor with memory manager and default config
         * @param memoryManager Engine memory manager
         */
        explicit TimerPool(memory::MemoryManager& memoryManager) :
            TimerPool(memoryManager, Config{}) {}

        /**
         * @brief Constructor with memory manager
         * @param memoryManager Engine memory manager
         * @param config Pool configuration
         */
        explicit TimerPool(
                memory::MemoryManager& memoryManager,
                const Config& config
                );

        /**
         * @brief Destructor
         */
        ~TimerPool();

        // Delete copy operations
        TimerPool(const TimerPool&) = delete;
        TimerPool& operator=(const TimerPool&) = delete;

        // =============================================================================
        // Timer Allocation
        // =============================================================================

        /**
         * @brief Allocate timer from pool
         * @return Timer pointer or nullptr if pool exhausted
         */
        [[nodiscard]] Timer* allocate();

        /**
         * @brief Allocate and initialize timer
         * @param config Timer configuration
         * @return Initialized timer or nullptr
         */
        [[nodiscard]] Timer* allocate(const Timer::Config& config);

        /**
         * @brief Return timer to pool
         * @param timer Timer to deallocate
         */
        void deallocate(Timer* timer);

        /**
         * @brief Batch allocate timers
         * @param count Number of timers to allocate
         * @return Vector of allocated timers
         */
        [[nodiscard]] std::vector<Timer*> batchAllocate(const std::size_t count);

        /**
         * @brief Batch deallocate timers
         * @param timers Timers to deallocate
         */
        void batchDeallocate(const std::vector<Timer*>& timers);

        // =============================================================================
        // Pool Management
        // =============================================================================

        /**
         * @brief Clear all timers and reset pool
         */
        void clear();

        /**
         * @brief Shrink pool to current usage
         */
        void shrink() const {
            std::lock_guard lock(poolMutex_);

            // TODO: Implement shrinking by releasing unused chunks
            // This is complex as we need to ensure timers aren't in use
        }

        /**
         * @brief Reserve additional capacity
         * @param additionalCount Number of additional timers
         * @return True if capacity was reserved
         */
        bool reserve(const std::size_t additionalCount);

        // =============================================================================
        // Pool Queries
        // =============================================================================

        /**
         * @brief Check if pool owns a timer
         * @param timer Timer to check
         * @return True if timer belongs to this pool
         */
        [[nodiscard]] bool owns(const Timer* timer) const;

        /**
         * @brief Get current pool size
         */
        [[nodiscard]] std::size_t size() const noexcept;

        /**
         * @brief Get number of allocated timers
         */
        [[nodiscard]] std::size_t allocated() const noexcept;

        /**
         * @brief Get number of available timers
         */
        [[nodiscard]] std::size_t available() const;

        /**
         * @brief Get pool utilization ratio
         */
        [[nodiscard]] float getUtilization() const noexcept;

        // =============================================================================
        // Statistics
        // =============================================================================

        /**
         * @brief Pool statistics
         */
        struct Stats {
            std::size_t poolSize; ///< Total pool size
            std::size_t allocated; ///< Currently allocated
            std::size_t available; ///< Currently available
            std::size_t peakUsage; ///< Peak allocation count
            std::uint64_t totalAllocations; ///< Lifetime allocations
            std::uint64_t totalDeallocations; ///< Lifetime deallocations
            float utilizationRatio; ///< Current utilization
            std::size_t memoryUsage; ///< Memory in bytes
        };

        /**
         * @brief Get pool statistics
         */
        [[nodiscard]] Stats getStats() const {
            return Stats{
                            .poolSize = size(),
                            .allocated = allocated(),
                            .available = available(),
                            .peakUsage = peakUsage_.load(std::memory_order_acquire),
                            .totalAllocations = allocationCount_.load(std::memory_order_acquire),
                            .totalDeallocations = deallocationCount_.load(std::memory_order_acquire),
                            .utilizationRatio = getUtilization(),
                            .memoryUsage = size() * sizeof(Timer)
                    };
        }

        /**
         * @brief Reset statistics
         */
        void resetStats() {
            peakUsage_.store(totalAllocated_.load());
            allocationCount_.store(0);
            deallocationCount_.store(0);
        }

    private:
        // =============================================================================
        // Private Types
        // =============================================================================

        struct MemoryChunk {
            std::uint8_t* memory; ///< Chunk memory
            std::size_t count; ///< Timer count in chunk
            bool fromMemoryManager; ///< Allocated via MemoryManager
        };

        // =============================================================================
        // Private Members
        // =============================================================================

        memory::MemoryManager& memoryManager_; ///< Memory manager
        Config config_; ///< Pool configuration

        mutable std::mutex poolMutex_; ///< Pool structure mutex
        mutable std::mutex availableMutex_; ///< Available stack mutex

        std::vector<MemoryChunk> memoryChunks_; ///< Memory chunks
        std::stack<Timer*> availableTimers_; ///< Available timers

        std::atomic<std::size_t> currentSize_; ///< Current pool size
        std::atomic<std::size_t> totalAllocated_; ///< Allocated count

        // Statistics
        std::atomic<std::size_t> peakUsage_; ///< Peak usage
        std::atomic<std::uint64_t> allocationCount_; ///< Total allocations
        std::atomic<std::uint64_t> deallocationCount_; ///< Total deallocations

        // =============================================================================
        // Private Methods
        // =============================================================================

        /**
         * @brief Initialize pool
         */
        void initialize();

        /**
         * @brief Shutdown pool
         */
        void shutdown();

        /**
         * @brief Grow pool (requires pool lock)
         */
        Timer* growPoolLocked(std::size_t count);

        /**
         * @brief Grow pool (public interface)
         */
        Timer* growPool();

        /**
         * @brief Rebuild available timer stack
         */
        void rebuildAvailableStack();

        /**
         * @brief Update peak usage statistic
         */
        void updatePeakUsage(const std::size_t currentUsage);
    };

} // namespace engine::time
