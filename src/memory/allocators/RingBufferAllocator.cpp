//
// Created by Andres Guerrero on 14-08-25.
//

#include "RingBufferAllocator.h"

#include <cassert>
#include <iostream>

namespace engine::memory {
    RingBufferAllocator::RingBufferAllocator(const MemorySize capacity, const char* name)
        : memory_(nullptr)
          , capacity_(capacity)
          , head_(0)
          , tail_(0)
          , fenceCounter_(0)
          , name_(name) {
        memory_ = std::aligned_alloc(CACHE_LINE_SIZE, capacity);

        if (!memory_) {
            std::cerr << "[RingBufferAllocator] Failed to allocate "
                << (capacity / (1024.0 * 1024.0)) << " MB for " << name_ << std::endl;
            throw std::bad_alloc();
        }

        // Initialize memory with debug pattern
#ifdef _DEBUG
        std::memset(memory_, 0xCD, capacity);
#endif

        stats_.currentUsage = 0;
        stats_.peakUsage = 0;
        stats_.totalAllocated = 0;
        stats_.allocationCount = 0;
    }

    RingBufferAllocator::~RingBufferAllocator() {
        if (!memory_) return;

#ifdef _DEBUG
        MemorySize used = getUsedMemory();
        if (used > 0) {
            std::cerr << "Warning: RingBufferAllocator '" << name_
                << "' destroyed with " << used
                << " bytes still allocated!" << std::endl;
        }
#endif

        std::free(memory_);
        memory_ = nullptr;
    }

    RingBufferAllocator::RingBufferAllocator(RingBufferAllocator&& other) noexcept
        : memory_(other.memory_)
          , capacity_(other.capacity_)
          , head_(other.head_.load())
          , tail_(other.tail_.load())
          , fenceCounter_(other.fenceCounter_.load())
          , name_(other.name_) {
        other.memory_ = nullptr;
        other.capacity_ = 0;
        other.head_ = 0;
        other.tail_ = 0;
        other.fenceCounter_ = 0;
    }

    RingBufferAllocator& RingBufferAllocator::operator=(RingBufferAllocator&& other) noexcept {
        if (this != &other) {
            // Free existing memory
            if (memory_) {
                std::free(memory_);
            }

            // Move from other
            memory_ = other.memory_;
            capacity_ = other.capacity_;
            head_ = other.head_.load();
            tail_ = other.tail_.load();
            fenceCounter_ = other.fenceCounter_.load();
            name_ = other.name_;

            // Reset other
            other.memory_ = nullptr;
            other.capacity_ = 0;
            other.head_ = 0;
            other.tail_ = 0;
            other.fenceCounter_ = 0;
        }
        return *this;
    }

    void* RingBufferAllocator::allocate(const MemorySize size, const MemorySize alignement,
                                        const AllocationFlags flags) {
        std::lock_guard lock(allocationMutex_);

        assert(isPowerOfTwo(alignement) && "Alignment msut be power of 2");

        constexpr MemorySize headerSize = sizeof(AllocationHeader);
        const MemorySize alignedHeaderSize = alignSize(headerSize, alignement);
        const MemorySize totalSize = alignedHeaderSize + size;

        MemorySize currentHead = head_.load(std::memory_order_acquire);
        const MemorySize currentTail = tail_.load(std::memory_order_acquire);

        MemorySize available;
        if (currentHead >= currentTail) {
            available = capacity_ - currentHead + currentTail;
        }
        else {
            available = currentTail - currentHead;
        }

        if (totalSize > available) {
            // Try to free old allocations by moving tail forward
            // In a real implementation, this would check fence values
            stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        MemorySize newHead = currentHead + totalSize;

        if (newHead >= capacity_) {
            if (totalSize > currentTail) {
                // Can't wrap - not enough space at beginning
                stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
                return nullptr;
            }

            currentHead = 0;
            newHead = totalSize;
        }

        void* headerPtr = static_cast<std::uint8_t*>(memory_) + currentHead;
        auto* header = static_cast<AllocationHeader*>(headerPtr);
        header->size = size;
        header->fence = fenceCounter_.load(std::memory_order_acquire);
#ifdef _DEBUG
        header->magic = MAGIC_NUMBER;
#endif

        void* userPtr = static_cast<std::uint8_t*>(headerPtr) + alignedHeaderSize;

        head_.store(newHead % capacity_, std::memory_order_release);

        if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
            std::memset(userPtr, 0, size);
        }
#ifdef _DEBUG
        else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
            std::memset(userPtr, 0xAB, size);
        }
#endif

        recordAllocation(totalSize);

        return userPtr;
    }

    void RingBufferAllocator::deallocate(void* ptr) {
        // Ring buffer doesn't support individual deallocation
        // Memory is freed when the tail advances past it
#ifdef _DEBUG
        if (ptr) {
            static bool warningShown = false;
            if (!warningShown) {
                std::cerr << "Warning: RingBufferAllocator does not support individual deallocation. "
                    << "Use fence system or reset()." << std::endl;
                warningShown = true;
            }
        }
#endif
    }

    MemorySize RingBufferAllocator::getUsedMemory() const {
        const MemorySize currentHead = head_.load(std::memory_order_acquire);
        const MemorySize currentTail = tail_.load(std::memory_order_acquire);

        if (currentHead >= currentTail) {
            return currentHead - currentTail;
        }

        return (capacity_ - currentTail) + currentHead;
    }

    void RingBufferAllocator::reset() {
        std::lock_guard lock(allocationMutex_);

#ifdef _DEBUG
        // Fill memory with uninitialized pattern
        std::memset(memory_, 0xCD, capacity_);
#endif

        head_.store(0, std::memory_order_release);
        tail_.store(0, std::memory_order_release);
        fenceCounter_.store(0, std::memory_order_release);

        stats_.currentUsage.store(0, std::memory_order_release);
    }

    bool RingBufferAllocator::owns(const void* ptr) const {
        if (!ptr || !memory_) return false;

        const auto* bytePtr = static_cast<const std::uint8_t*>(ptr);
        const auto* memStart = static_cast<const std::uint8_t*>(memory_);
        const std::uint8_t* memEnd = memStart + capacity_;

        return bytePtr >= memStart && bytePtr < memEnd;
    }

    MemorySize RingBufferAllocator::getAllocationSize(const void* ptr) const {
        if (!owns(ptr)) return 0;

#ifdef _DEBUG
        // Try to find the header before this allocation
        const std::uint8_t* bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);

        // Search backwards for the header (within reasonable distance)
        for (MemorySize offset = sizeof(AllocationHeader); offset <= 256; offset += alignof(AllocationHeader)) {
            const AllocationHeader* header = reinterpret_cast<const AllocationHeader*>(bytePtr - offset);

            if (header->magic == MAGIC_NUMBER) {
                return header->size;
            }
        }
#endif

        return 0;
    }

    std::uint64_t RingBufferAllocator::createFence() {
        return fenceCounter_.fetch_add(1, std::memory_order_acq_rel) + 1;
    }

    void RingBufferAllocator::waitForFence(std::uint64_t fence) {
        std::lock_guard lock(allocationMutex_);

        // Move tail forward to the position marked by the fence
        // This would free all allocations before the fence
        // Implementation depends on how fences are tracked with allocations

        // For now, this is a simplified implementation
        // In production, you'd track fence values with each allocation
    }

    bool RingBufferAllocator::canAllocateWithoutWrap(const MemorySize size) const {
        const MemorySize currentHead = head_.load(std::memory_order_acquire);

        return (currentHead + size) <= capacity_;
    }
}
