/**
 * @file TimerQueue.h
 * @brief High-performance priority queue for timer management
 * @details Optimized timer queue implementation using binary heap with O(log n)
 *          insertion and removal. Supports thousands of concurrent timers with
 *          minimal memory overhead and cache-friendly access patterns.
 *
 * @author Development Team
 * @date Created on 2024-01-20
 */

#pragma once

#include "../core/TimeTypes.h"
#include "../timers/Timer.h"
#include "../timers/TimerHandle.h"

#include "../../memory/MemorySystem.h"

#include <vector>
#include <unordered_map>
#include <mutex>
#include <atomic>
#include <functional>

namespace engine::time {

    // =============================================================================
    // Timer Queue Configuration
    // =============================================================================

    /**
     * @brief Configuration for timer queue
     */
    struct TimerQueueConfig {
        std::size_t initialCapacity{256}; ///< Initial queue capacity
        std::size_t maxCapacity{8192}; ///< Maximum timer capacity
        bool allowDynamicGrowth{true}; ///< Allow queue to grow
        float growthFactor{1.5f}; ///< Growth multiplier

        // Memory settings
        memory::MemoryManager* memoryManager{nullptr};
        memory::MemoryCategory memoryCategory{
                        memory::MemoryCategory::GENERAL
                };

        // Performance settings
        bool useLockFreeOperations{false}; ///< Use lock-free algorithms
        bool batchProcessing{true}; ///< Process timers in batches
        std::size_t batchSize{16}; ///< Batch processing size

        // Debug settings
        bool trackStatistics{true}; ///< Track queue statistics
        bool validateHeapProperty{false}; ///< Validate heap in debug
    };

    // =============================================================================
    // Timer Queue Statistics
    // =============================================================================

    /**
     * @brief Non-atomic statistics snapshot
     * @details Plain data structure for returning statistics safely
     */
    struct TimerQueueStatsSnapshot {
        std::size_t currentSize{0};
        std::size_t peakSize{0};
        std::uint64_t totalInsertions{0};
        std::uint64_t totalRemovals{0};
        std::uint64_t totalProcessed{0};

        Duration totalProcessingTime{};
        Duration averageInsertTime{};
        Duration averageRemoveTime{};

        std::uint32_t heapResizes{0};
        std::uint32_t heapRebuilds{0};
    };

    /**
     * @brief Atomic statistics for thread-safe updates
     */
    struct TimerQueueStats {
        std::atomic<std::size_t> currentSize{0}; ///< Current timer count
        std::atomic<std::size_t> peakSize{0}; ///< Peak timer count
        std::atomic<std::uint64_t> totalInsertions{0}; ///< Total timers added
        std::atomic<std::uint64_t> totalRemovals{0}; ///< Total timers removed
        std::atomic<std::uint64_t> totalProcessed{0}; ///< Total timers fired

        std::atomic<Duration> totalProcessingTime{}; ///< Total processing time
        std::atomic<Duration> averageInsertTime{}; ///< Average insert time
        std::atomic<Duration> averageRemoveTime{}; ///< Average remove time

        std::atomic<std::uint32_t> heapResizes{0}; ///< Number of resizes
        std::atomic<std::uint32_t> heapRebuilds{0}; ///< Heap rebuild count


        /**
         * @brief Create snapshot of current statistics
         * @return Non-atomic copy of statistics
         */
        [[nodiscard]] TimerQueueStatsSnapshot snapshot() const noexcept {
            return TimerQueueStatsSnapshot{
                .currentSize = currentSize.load(std::memory_order_relaxed),
                .peakSize = peakSize.load(std::memory_order_relaxed),
                .totalInsertions = totalInsertions.load(std::memory_order_relaxed),
                .totalRemovals = totalRemovals.load(std::memory_order_relaxed),
                .totalProcessed = totalProcessed.load(std::memory_order_relaxed),
                .totalProcessingTime = totalProcessingTime.load(std::memory_order_relaxed),
                .averageInsertTime = averageInsertTime.load(std::memory_order_relaxed),
                .averageRemoveTime = averageRemoveTime.load(std::memory_order_relaxed),
                .heapResizes = heapResizes.load(std::memory_order_relaxed),
                .heapRebuilds = heapRebuilds.load(std::memory_order_relaxed)
            };
        }

        void reset() noexcept {
            currentSize.store(0, std::memory_order_relaxed);
            peakSize.store(0, std::memory_order_relaxed);
            totalInsertions.store(0, std::memory_order_relaxed);
            totalRemovals.store(0, std::memory_order_relaxed);
            totalProcessed.store(0, std::memory_order_relaxed);
            totalProcessingTime.store(Duration::zero(), std::memory_order_relaxed);
            averageInsertTime.store(Duration::zero(), std::memory_order_relaxed);
            averageRemoveTime.store(Duration::zero(), std::memory_order_relaxed);
            heapResizes.store(0, std::memory_order_relaxed);
            heapRebuilds.store(0, std::memory_order_relaxed);
        }
    };

    // =============================================================================
    // Timer Queue Class
    // =============================================================================

    /**
     * @brief Priority queue for efficient timer management
     * @details Min-heap implementation optimized for timer expiration ordering.
     *          Provides O(log n) insertion/removal with cache-friendly layout.
     */
    class TimerQueue {
    public:
        /**
         * @brief Constructor with configuration
         * @param config Queue configuration
         */
        explicit TimerQueue(const TimerQueueConfig& config = {});

        /**
         * @brief Destructor
         */
        ~TimerQueue();

        // Delete copy operations
        TimerQueue(const TimerQueue&) = delete;
        TimerQueue& operator=(const TimerQueue&) = delete;

        // Allow move operations
        TimerQueue(TimerQueue&& other) noexcept;
        TimerQueue& operator=(TimerQueue&& other) noexcept;

        // =============================================================================
        // Initialization
        // =============================================================================

        /**
         * @brief Initialize queue with memory manager
         * @param memoryManager Memory manager instance
         * @return True if initialization successful
         */
        bool initialize(memory::MemoryManager* memoryManager);

        /**
         * @brief Shutdown and cleanup
         */
        void shutdown();

        /**
         * @brief Check if initialized
         */
        [[nodiscard]] bool isInitialized() const noexcept {
            return initialized_;
        }

        // =============================================================================
        // Timer Operations
        // =============================================================================

        /**
         * @brief Insert timer into queue
         * @param timer Timer to insert
         * @return True if insertion successful
         */
        bool insert(const Timer& timer);

        /**
         * @brief Remove timer from queue
         * @param handle Timer handle
         * @return True if removal successful
         */
        bool remove(const TimerHandle& handle);

        /**
         * @brief Update timer in queue
         * @param handle Timer handle
         * @param newExpiration New expiration time
         * @return True if update successful
         */
        bool update(const TimerHandle& handle, TimeStamp newExpiration);

        /**
         * @brief Peek at next timer without removing
         * @return Pointer to next timer or nullptr if empty
         */
        [[nodiscard]] const Timer* peek() const noexcept;

        /**
         * @brief Pop next timer from queue
         * @return Next timer or nullopt if empty
         */
        [[nodiscard]] std::optional<Timer> pop();

        /**
         * @brief Process expired timers
         * @param currentTime Current timestamp
         * @param callback Callback for each expired timer
         * @return Number of timers processed
         */
        std::size_t processExpired(
                TimeStamp currentTime,
                const std::function<void(Timer&)>& callback
                );

        /**
         * @brief Process expired timers in batches
         * @param currentTime Current timestamp
         * @param batchCallback Callback for timer batch
         * @return Number of timers processed
         */
        std::size_t processBatch(
                TimeStamp currentTime,
                const std::function<void(std::vector<Timer>&)>& batchCallback
                );

        // =============================================================================
        // Queue Management
        // =============================================================================

        /**
         * @brief Clear all timers
         */
        void clear();

        /**
         * @brief Reserve capacity
         * @param capacity New capacity
         */
        void reserve(std::size_t capacity);

        /**
         * @brief Rebuild heap structure
         * @details Restores heap property after bulk operations
         */
        void rebuild();

        /**
         * @brief Validate heap property
         * @return True if heap is valid
         */
        [[nodiscard]] bool isValid() const;

        // =============================================================================
        // Query Operations
        // =============================================================================

        /**
         * @brief Get queue size
         */
        [[nodiscard]] std::size_t size() const noexcept {
            return heap_.size();
        }

        /**
         * @brief Check if queue is empty
         */
        [[nodiscard]] bool empty() const noexcept {
            return heap_.empty();
        }

        /**
         * @brief Get queue capacity
         */
        [[nodiscard]] std::size_t capacity() const noexcept {
            return heap_.capacity();
        }

        /**
         * @brief Find timer by handle
         * @param handle Timer handle
         * @return Pointer to timer or nullptr if not found
         */
        [[nodiscard]] Timer* find(const TimerHandle& handle);
        [[nodiscard]] const Timer* find(const TimerHandle& handle) const;

        /**
         * @brief Check if timer exists
         * @param handle Timer handle
         * @return True if timer exists in queue
         */
        [[nodiscard]] bool contains(const TimerHandle& handle) const;

        /**
         * @brief Get time until next timer expires
         * @param currentTime Current timestamp
         * @return Duration until next expiration or max if empty
         */
        [[nodiscard]] Duration getTimeUntilNext(TimeStamp currentTime) const;

        // =============================================================================
        // Statistics
        // =============================================================================

        /**
         * @brief Get queue statistics
         */
        [[nodiscard]] TimerQueueStatsSnapshot getStats() const noexcept {
            return stats_.snapshot();
        }

        /**
         * @brief Reset statistics
         */
        void resetStats() noexcept {
            stats_.reset();
        }

        // =============================================================================
        // Configuration
        // =============================================================================

        /**
         * @brief Get configuration
         */
        [[nodiscard]] const TimerQueueConfig& getConfig() const noexcept {
            return config_;
        }

        /**
         * @brief Update configuration
         * @param config New configuration
         */
        void updateConfig(const TimerQueueConfig& config);

    private:
        // =============================================================================
        // Internal Types
        // =============================================================================

        /**
         * @brief Queue entry with timer and metadata
         */
        struct QueueEntry {
            Timer timer; ///< Timer instance
            std::size_t heapIndex{}; ///< Index in heap

            // Constructors
            QueueEntry() = default;

            // Delete copy operations (Timer is not copyable)
            QueueEntry(const QueueEntry&) = delete;
            QueueEntry& operator=(const QueueEntry&) = delete;

            // Explicitly default move operations
            QueueEntry(QueueEntry&&) noexcept = default;
            QueueEntry& operator=(QueueEntry&&) noexcept = default;

            /**
             * @brief Comparison for min-heap ordering
             */
            bool operator<(const QueueEntry& other) const noexcept {
                return timer.getExpiration() > other.timer.getExpiration();
            }
        };

        // =============================================================================
        // Member Variables
        // =============================================================================

        TimerQueueConfig config_; ///< Configuration
        bool initialized_{false}; ///< Initialization state

        std::vector<QueueEntry> heap_; ///< Min-heap storage
        std::unordered_map<TimerID, std::size_t> indexMap_; ///< Handle to index map

        mutable std::mutex queueMutex_; ///< Thread safety
        TimerQueueStats stats_; ///< Statistics

        memory::MemoryManager* memoryManager_{nullptr}; ///< Memory manager

        // =============================================================================
        // Heap Operations
        // =============================================================================

        /**
         * @brief Move element up to restore heap property
         * @param index Element index
         */
        void heapifyUp(std::size_t index);

        /**
         * @brief Move element down to restore heap property
         * @param index Element index
         */
        void heapifyDown(std::size_t index);

        /**
         * @brief Get parent index
         */
        [[nodiscard]] static constexpr std::size_t getParent(const std::size_t index) noexcept {
            return (index - 1) / 2;
        }

        /**
         * @brief Get left child index
         */
        [[nodiscard]] static constexpr std::size_t getLeftChild(const std::size_t index) noexcept {
            return 2 * index + 1;
        }

        /**
         * @brief Get right child index
         */
        [[nodiscard]] static constexpr std::size_t getRightChild(const std::size_t index) noexcept {
            return 2 * index + 2;
        }

        /**
         * @brief Swap two elements and update indices
         */
        void swapElements(std::size_t i, std::size_t j);

        /**
         * @brief Update index map for element
         */
        void updateIndexMap(std::size_t index);

        /**
         * @brief Grow heap capacity
         */
        bool growCapacity();

        /**
         * @brief Update statistics for operation
         */
        void updateStats(bool isInsertion, Duration operationTime);
    };

} // namespace engine::time
