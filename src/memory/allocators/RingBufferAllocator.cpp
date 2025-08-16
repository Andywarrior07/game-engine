//
// Created by Andres Guerrero on 14-08-25.
//

#include "RingBufferAllocator.h"

#include <cassert>

namespace engine::memory {
    RingBufferAllocator::RingBufferAllocator(const MemorySize capacity, const char* name)
        : memory(nullptr)
          , capacity(capacity)
          , head(0)
          , tail(0)
          , fenceCounter(0)
          , name(name) {
        memory = std::aligned_alloc(CACHE_LINE_SIZE, capacity);

        if (!memory) {
            throw std::bad_alloc();
        }

        // Initialize memory with debug pattern
#ifdef _DEBUG
        std::memset(memory, 0xCD, capacity);
#endif

        stats.currentUsage = 0;
        stats.peakUsage = 0;
        stats.totalAllocated = 0;
        stats.allocationCount = 0;
    }

    RingBufferAllocator::~RingBufferAllocator() {
        if (!memory) return;

#ifdef _DEBUG
        MemorySize used = getUsedMemory();
        if (used > 0) {
            std::cerr << "Warning: RingBufferAllocator '" << name
                << "' destroyed with " << used
                << " bytes still allocated!" << std::endl;
        }
#endif

        std::free(memory);
        memory = nullptr;
    }

    RingBufferAllocator::RingBufferAllocator(RingBufferAllocator&& other) noexcept
        : memory(other.memory)
          , capacity(other.capacity)
          , head(other.head.load())
          , tail(other.tail.load())
          , fenceCounter(other.fenceCounter.load())
          , name(other.name) {
        other.memory = nullptr;
        other.capacity = 0;
        other.head = 0;
        other.tail = 0;
        other.fenceCounter = 0;
    }

    RingBufferAllocator& RingBufferAllocator::operator=(RingBufferAllocator&& other) noexcept {
        if (this != &other) {
            // Free existing memory
            if (memory) {
                std::free(memory);
            }

            // Move from other
            memory = other.memory;
            capacity = other.capacity;
            head = other.head.load();
            tail = other.tail.load();
            fenceCounter = other.fenceCounter.load();
            name = other.name;

            // Reset other
            other.memory = nullptr;
            other.capacity = 0;
            other.head = 0;
            other.tail = 0;
            other.fenceCounter = 0;
        }
        return *this;
    }

    void* RingBufferAllocator::allocate(const MemorySize size, const MemorySize alignement,
                                        const AllocationFlags flags) {
        std::lock_guard lock(allocationMutex);

        assert(isPowerOfTwo(alignement) && "Alignment msut be power of 2");

        constexpr MemorySize headerSize = sizeof(AllocationHeader);
        const MemorySize alignedHeaderSize = alignSize(headerSize, alignement);
        const MemorySize totalSize = alignedHeaderSize + size;

        MemorySize currentHead = head.load(std::memory_order_acquire);
        const MemorySize currentTail = tail.load(std::memory_order_acquire);

        MemorySize available;
        if (currentHead >= currentTail) {
            available = capacity - currentHead + currentTail;
        }
        else {
            available = currentTail - currentHead;
        }

        if (totalSize > available) {
            // Try to free old allocations by moving tail forward
            // In a real implementation, this would check fence values
            stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        MemorySize newHead = currentHead + totalSize;

        if (newHead >= capacity) {
            if (totalSize > currentTail) {
                // Can't wrap - not enough space at beginning
                stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
                return nullptr;
            }

            currentHead = 0;
            newHead = totalSize;
        }

        void* headerPtr = static_cast<std::uint8_t*>(memory) + currentHead;
        auto* header = static_cast<AllocationHeader*>(headerPtr);
        header->size = size;
        header->fence = fenceCounter.load(std::memory_order_acquire);
#ifdef _DEBUG
        header->magic = MAGIC_NUMBER;
#endif

        void* userPtr = static_cast<std::uint8_t*>(headerPtr) + alignedHeaderSize;

        head.store(newHead % capacity, std::memory_order_release);

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
        const MemorySize currentHead = head.load(std::memory_order_acquire);
        const MemorySize currentTail = tail.load(std::memory_order_acquire);

        if (currentHead >= currentTail) {
            return currentHead - currentTail;
        }

        return (capacity - currentTail) + currentHead;
    }

    void RingBufferAllocator::reset() {
        std::lock_guard lock(allocationMutex);

#ifdef _DEBUG
        // Fill memory with uninitialized pattern
        std::memset(memory, 0xCD, capacity);
#endif

        head.store(0, std::memory_order_release);
        tail.store(0, std::memory_order_release);
        fenceCounter.store(0, std::memory_order_release);

        stats.currentUsage.store(0, std::memory_order_release);
    }

    bool RingBufferAllocator::owns(const void* ptr) const {
        if (!ptr || !memory) return false;

        const auto* bytePtr = static_cast<const std::uint8_t*>(ptr);
        const auto* memStart = static_cast<const std::uint8_t*>(memory);
        const std::uint8_t* memEnd = memStart + capacity;

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
        return fenceCounter.fetch_add(1, std::memory_order_acq_rel) + 1;
    }

    void RingBufferAllocator::waitForFence(std::uint64_t fence) {
        std::lock_guard lock(allocationMutex);

        // Move tail forward to the position marked by the fence
        // This would free all allocations before the fence
        // Implementation depends on how fences are tracked with allocations

        // For now, this is a simplified implementation
        // In production, you'd track fence values with each allocation
    }

    bool RingBufferAllocator::canAllocateWithoutWrap(const MemorySize size) const {
        const MemorySize currentHead = head.load(std::memory_order_acquire);

        return (currentHead + size) <= capacity;
    }
}
