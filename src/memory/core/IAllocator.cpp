//
// Created by Andres Guerrero on 12-08-25.
//

#include "IAllocator.h"

namespace engine::memory {
    void* IAllocator::reallocate(void* ptr, const MemorySize newSize, const MemorySize alignment) {
        // Default implementation: allocate new, copy, free old
        if (!ptr) {
            return allocate(newSize, alignment);
        }

        if (newSize == 0) {
            deallocate(ptr);
            return nullptr;
        }

        void* newPtr = allocate(newSize, alignment);
        if (newPtr && ptr) {
            // Get original size if tracking is enabled
            MemorySize oldSize = getAllocationSize(ptr);
            std::memcpy(newPtr, ptr, std::min(oldSize, newSize));
            deallocate(ptr);
        }

        return newPtr;
    }

    MemorySize IAllocator::getFreeMemory() const {
        return getCapacity() - getUsedMemory();
    }

    void* IAllocator::alignPointer(void* ptr, MemorySize alignment) {
        const uintptr_t mask = alignment - 1;
        const auto misalignment = reinterpret_cast<uintptr_t>(ptr) & mask;
        const MemorySize adjustment = misalignment ? (alignment - misalignment) : 0;

        return reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(ptr) + adjustment);
    }

    MemorySize IAllocator::alignSize(const MemorySize size, const MemorySize alignment) {
        const MemorySize mask = alignment - 1;

        return (size + mask) & ~mask;
    }

    bool IAllocator::isPowerOfTwo(const MemorySize value) {
        return value && !(value & (value - 1));
    }

    void IAllocator::recordAllocation(const MemorySize size, const MemoryCategory category) const {
        stats_.totalAllocated.fetch_add(size, std::memory_order_relaxed);
        stats_.currentUsage.fetch_add(size, std::memory_order_relaxed);
        stats_.allocationCount.fetch_add(1, std::memory_order_relaxed);
        stats_.categoryUsage[static_cast<std::size_t>(category)].fetch_add(size, std::memory_order_relaxed);

        // Update peak usage
        const MemorySize current = stats_.currentUsage.load(std::memory_order_relaxed);
        MemorySize peak = stats_.peakUsage.load(std::memory_order_relaxed);
        while (current > peak && !stats_.peakUsage.compare_exchange_weak(peak, current)) {
            // Keep trying until we update peak or current is no longer greater
        }
    }

    // Update statistics on deallocation
    void IAllocator::recordDeallocation(const MemorySize size, const MemoryCategory category) const {
        stats_.totalFreed.fetch_add(size, std::memory_order_relaxed);
        stats_.currentUsage.fetch_sub(size, std::memory_order_relaxed);
        stats_.categoryUsage[static_cast<std::size_t>(category)].fetch_sub(size, std::memory_order_relaxed);
    }
}
