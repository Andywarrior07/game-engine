//
// Created by Andres Guerrero on 19-09-25.
//

#pragma once

#include "../core/IAllocator.h"

#include <atomic>
#include <mutex>
#include <cassert>
#include <iostream>
#include <cstring>

namespace engine::memory {
    /**
     * @brief Circular buffer allocator that automatically overwrites old data
     * @details Optimized for time-series data, frame history, and rolling metrics.
     *          Uses a circular buffer strategy where new allocations overwrite the oldest data
     *          when the buffer is full. Thread-safe with atomic operations for read/write heads.
     *
     * Performance characteristics:
     * - O(1) allocation time
     * - Cache-friendly sequential access patterns
     * - Lock-free for single producer/consumer scenarios
     * - Automatic memory reclamation
     *
     * Ideal for:
     * - Frame time history (last N frames)
     * - Performance metrics rolling window
     * - Event logs with fixed history size
     * - Profiling data collection
     */
    class CircularBufferAllocator final : public IAllocator {
    public:
        /**
         * @brief Construct circular buffer allocator
         * @param capacity Total buffer capacity in bytes
         * @param maxEntries Maximum number of entries (for tracking)
         * @param name Debug name for the allocator
         */
        explicit CircularBufferAllocator(MemorySize capacity,
                                         std::size_t maxEntries = 0,
                                         const char* name = "CircularBufferAllocator");

        /**
         * @brief Destructor - ensures proper cleanup
         */
        ~CircularBufferAllocator() override;

        // Disable copy semantics
        CircularBufferAllocator(const CircularBufferAllocator&) = delete;
        CircularBufferAllocator& operator=(const CircularBufferAllocator&) = delete;

        /**
         * @brief Move constructor
         */
        CircularBufferAllocator(CircularBufferAllocator&& other) noexcept;

        /**
         * @brief Move assignment operator
         */
        CircularBufferAllocator& operator=(CircularBufferAllocator&& other) noexcept;

        /**
         * @brief Allocate memory in circular buffer
         * @param size Size of allocation in bytes
         * @param alignment Memory alignment requirement
         * @param flags Allocation flags
         * @return Pointer to allocated memory or nullptr if size too large
         */
        void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                       AllocationFlags flags = AllocationFlags::NONE) override;

        /**
         * @brief Deallocate memory (no-op for circular buffer)
         * @param ptr Pointer to deallocate
         */
        void deallocate(void* ptr) override;

        /**
         * @brief Get total capacity of the buffer
         */
        MemorySize getCapacity() const override {
            return capacity_;
        }

        /**
         * @brief Get currently used memory (approximate)
         */
        MemorySize getUsedMemory() const override;

        /**
         * @brief Reset the circular buffer
         */
        void reset() override;

        /**
         * @brief Check if buffer owns a pointer
         */
        bool owns(const void* ptr) const override;

        /**
         * @brief Get allocation size for a pointer
         */
        MemorySize getAllocationSize(const void* ptr) const override;

        /**
         * @brief Get allocator name
         */
        const char* getName() const override {
            return name_;
        }

        // Circular buffer specific methods

        /**
         * @brief Get the oldest available entry
         * @return Pointer to oldest entry or nullptr if buffer empty
         */
        void* getOldestEntry();

        /**
         * @brief Advance read head to next entry
         */
        void advanceReadHead();

        /**
         * @brief Get number of times data was overwritten
         */
        std::size_t getOverwriteCount() const {
            return overwriteCount_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get total number of entries written
         */
        std::size_t getTotalEntryCount() const {
            return entryCount_.load(std::memory_order_acquire);
        }

        /**
         * @brief Check if buffer is full (write head caught up to read head)
         */
        bool isFull() const {
            return static_cast<float>(getUsedMemory()) >= (static_cast<float>(capacity_) * 0.9f); // 90% threshold
        }

        /**
         * @brief Get utilization percentage
         */
        float getUtilization() const {
            return static_cast<float>(getUsedMemory()) / static_cast<float>(capacity_);
        }

    private:
        /**
         * @brief Entry header stored before each allocation
         */
        struct EntryHeader {
            MemorySize size{}; ///< User data size
            MemorySize totalSize{}; ///< Total size including header
            std::chrono::steady_clock::time_point timestamp; ///< Allocation timestamp
            std::uint64_t entryId{}; ///< Unique entry identifier
#ifdef _DEBUG
            std::uint32_t magic; ///< Magic number for validation
#endif
        };

        /**
         * @brief Entry tracking information
         */
        struct EntryInfo {
            MemorySize offset; ///< Offset in buffer
            MemorySize size; ///< Total size
            std::uint64_t entryId; ///< Entry identifier
            bool valid; ///< Entry is valid
        };

        void* memory_; ///< Buffer memory
        MemorySize capacity_; ///< Total buffer capacity
        std::size_t maxEntries_; ///< Maximum tracked entries
        std::atomic<MemorySize> writeHead_; ///< Current write position
        std::atomic<MemorySize> readHead_; ///< Current read position
        std::atomic<std::size_t> entryCount_; ///< Total entries written
        std::atomic<std::size_t> overwriteCount_; ///< Number of overwrites
        const char* name_; ///< Debug name

        std::vector<EntryInfo> entries_; ///< Entry tracking array
        std::mutex allocationMutex_; ///< Synchronization mutex

#ifdef _DEBUG
        static constexpr std::uint32_t MAGIC_NUMBER = 0xC1RCBUFF;
#endif

        /**
         * @brief Advance read head by specified amount or to next entry
         * @param minAdvance Minimum bytes to advance
         */
        void advanceReadHead(MemorySize minAdvance);
    };
}
