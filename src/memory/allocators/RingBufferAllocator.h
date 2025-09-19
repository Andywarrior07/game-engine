//
// Created by Andres Guerrero on 14-08-25.
//

#pragma once
#include <mutex>

#include "../core/IAllocator.h"

namespace engine::memory {
    class RingBufferAllocator final : public IAllocator {
    public:
        explicit RingBufferAllocator(MemorySize capacity, const char* name = "RingBufferAllocator");
        ~RingBufferAllocator() override;

        RingBufferAllocator(const RingBufferAllocator&) = delete;
        RingBufferAllocator& operator=(const RingBufferAllocator&) = delete;
        RingBufferAllocator(RingBufferAllocator&& other) noexcept;
        RingBufferAllocator& operator=(RingBufferAllocator&& other) noexcept;

        void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                       AllocationFlags flags = AllocationFlags::NONE) override;
        void deallocate(void* ptr) override;
        MemorySize getCapacity() const override { return capacity_; }
        MemorySize getUsedMemory() const override;
        void reset() override;
        bool owns(const void* ptr) const override;
        MemorySize getAllocationSize(const void* ptr) const override;
        const char* getName() const override { return name_; }

        std::uint64_t createFence();
        void waitForFence(std::uint64_t fence);
        bool canAllocateWithoutWrap(MemorySize size) const;

        /**
 * @brief Get current write offset in the ring buffer
 * @return Current head position (write offset) in bytes from buffer start
 *
 * This method is crucial for temporal operations like:
 * - Input history tracking
 * - Frame synchronization in networking
 * - Rollback/replay systems
 * - Debug temporal navigation
 */
        MemorySize getCurrentOffset() const noexcept {
            return head_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get current read offset in the ring buffer
         * @return Current tail position (read offset) in bytes from buffer start
         */
        MemorySize getReadOffset() const noexcept {
            return tail_.load(std::memory_order_acquire);
        }

        /**
         * @brief Get offset range currently in use
         * @return Pair of {tail_offset, head_offset} representing the active range
         *
         * Useful for:
         * - Determining valid history range
         * - Memory usage visualization
         * - Debugging buffer state
         */
        std::pair<MemorySize, MemorySize> getActiveRange() const noexcept {
            const MemorySize currentTail = tail_.load(std::memory_order_acquire);
            const MemorySize currentHead = head_.load(std::memory_order_acquire);
            return {currentTail, currentHead};
        }

        /**
         * @brief Check if a given offset is still valid (not overwritten)
         * @param offset Offset to check
         * @return true if offset is within the active range
         *
         * Critical for input history validation:
         * - Ensures we don't access overwritten data
         * - Validates historical input queries
         */
        bool isOffsetValid(const MemorySize offset) const noexcept {
            const auto [tailOffset, headOffset] = getActiveRange();

            if (tailOffset <= headOffset) {
                // Linear case: [tail...head]
                return offset >= tailOffset && offset < headOffset;
            }

            // Wrapped case: [0...head] + [tail...capacity]
            return offset >= tailOffset || offset < headOffset;
        }

        /**
         * @brief Calculate distance between two offsets in ring buffer
         * @param fromOffset Starting offset
         * @param toOffset Ending offset
         * @return Distance in bytes, accounting for wrap-around
         */
        MemorySize getOffsetDistance(const MemorySize fromOffset, const MemorySize toOffset) const noexcept {
            if (toOffset >= fromOffset) {
                return toOffset - fromOffset;
            }

            // Wrapped case
            return (capacity_ - fromOffset) + toOffset;
        }

    private:
        struct AllocationHeader {
            MemorySize size;
            std::uint64_t fence;
#ifdef _DEBUG
            std::uint32_t magic; // Magic number for validation
#endif
        };

        void* memory_;
        MemorySize capacity_;
        std::atomic<MemorySize> head_;
        std::atomic<MemorySize> tail_;
        std::atomic<std::uint64_t> fenceCounter_;
        const char* name_;

        std::mutex allocationMutex_;

#ifdef _DEBUG
        static constexpr std::uint32_t MAGIC_NUMBER = 0xFEEDFACE;
#endif
    };
}
