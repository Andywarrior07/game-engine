//
// Created by Andres Guerrero on 14-08-25.
//

#include "LinearAllocator.h"

#include <cassert>

namespace engine::memory {
    LinearAllocator::LinearAllocator(const MemorySize capacity, const char* name)
        : memory(nullptr)
          , capacity(capacity)
          , current(0)
          , allocationCount(0)
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

    LinearAllocator::~LinearAllocator() {
        if (!memory) return;


#ifdef _DEBUG
        if (current.load() > 0) {
            std::cerr << "Warning: LinearAllocator '" << name_
                << "' destroyed with " << current_.load()
                << " bytes still allocated!" << std::endl;
        }
#endif

        std::free(memory);
        memory = nullptr;
    }

    LinearAllocator::LinearAllocator(LinearAllocator&& other) noexcept
        : memory(other.memory)
          , capacity(other.capacity)
          , current(other.current.load())
          , allocationCount(other.allocationCount.load())
          , name(other.name) {
        other.memory = nullptr;
        other.capacity = 0;
        other.current = 0;
        other.allocationCount = 0;

#ifdef _DEBUG
        std::lock_guard<std::mutex> lock(other.debugMutex);
        debugAllocations = std::move(other.debugAllocations);
#endif
    }

    LinearAllocator& LinearAllocator::operator=(LinearAllocator&& other) noexcept {
        if (this == &other) return *this;

        if (memory) {
            std::free(memory);
        }

        memory = other.memory;
        capacity = other.capacity;
        current = other.current.load();
        allocationCount = other.allocationCount.load();
        name = other.name;

        other.memory = nullptr;
        other.capacity = 0;
        other.current = 0;
        other.allocationCount = 0;

#ifdef _DEBUG
        std::lock_guard<std::mutex> lock(other.debugMutex);
        debugAllocations = std::move(other.debugAllocations);
#endif

        return *this;
    }

    void* LinearAllocator::allocate(const MemorySize size, const MemorySize alignment, const AllocationFlags flags) {
        assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");

        MemorySize currentPos = current.load(std::memory_order_acquire);

        void* currentAddress = static_cast<std::uint8_t*>(memory) + currentPos;
        void* alignedAddress = alignPointer(currentAddress, alignment);

        const MemorySize adjustment = static_cast<std::uint8_t*>(alignedAddress) -
            static_cast<std::uint8_t*>(currentAddress);

        MemorySize newPos = currentPos + adjustment + size;

        if (newPos > capacity) {
            stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        MemorySize expected = currentPos;
        while (!current.compare_exchange_weak(expected, newPos,
                                              std::memory_order_acq_rel,
                                              std::memory_order_acquire)) {
            currentPos = expected;
            currentAddress = static_cast<std::uint8_t*>(memory) + currentPos;
            alignedAddress = alignPointer(currentAddress, alignment);
            const MemorySize newAdjustment = static_cast<std::uint8_t*>(alignedAddress) -
                static_cast<std::uint8_t*>(currentAddress);
            const MemorySize newNewPos = currentPos + newAdjustment + size;

            if (newNewPos > capacity) {
                stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
                return nullptr;
            }

            newPos = newNewPos;
        }

        allocationCount.fetch_add(1, std::memory_order_relaxed);

#ifdef _DEBUG
        // Track allocation for debugging
        {
            std::lock_guard<std::mutex> lock(debugMutex);
            debugAllocations.push_back({currentPos + adjustment, size});
        }
#endif

        if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
            std::memset(alignedAddress, 0, size);
        }
#ifdef _DEBUG
        else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
            std::memset(alignedAddress, 0xAB, size);
        }
#endif

        recordAllocation(size + adjustment);

        return alignedAddress;
    }

    void LinearAllocator::deallocate(void* ptr) {
        if (!ptr) return;

        // Linear allocator doesn't support individual deallocation
        // This is a no-op
#ifdef _DEBUG
        // In debug mode, warn about attempted deallocation
        static bool warningShown = false;
        if (!warningShown) {
            std::cerr << "Warning: LinearAllocator does not support individual deallocation. "
                << "Use reset() to free all memory." << std::endl;
            warningShown = true;
        }
#endif
    }

    MemorySize LinearAllocator::getUsedMemory() const {
        return current.load(std::memory_order_acquire);
    }

    void LinearAllocator::reset() {
#ifdef _DEBUG
        // Fill freed memory with pattern
        MemorySize used = current.load(std::memory_order_acquire);
        if (used > 0) {
            std::memset(memory, 0xCD, used);
        }

        // Clear debug allocations
        {
            std::lock_guard<std::mutex> lock(debugMutex);
            debugAllocations.clear();
        }
#endif

        current.store(0, std::memory_order_release);
        allocationCount.store(0, std::memory_order_release);

        stats.currentUsage.store(0, std::memory_order_release);
    }

    bool LinearAllocator::owns(const void* ptr) const {
        if (!ptr || !memory) return false;

        const auto* bytePtr = static_cast<const std::uint8_t*>(ptr);
        const auto* memStart = static_cast<const std::uint8_t*>(memory);
        const std::uint8_t* memEnd = memStart + capacity;

        return bytePtr >= memStart && bytePtr < memEnd;
    }

    MemorySize LinearAllocator::getAllocationSize(const void* ptr) const {
        if (!owns(ptr)) return 0;

#ifdef _DEBUG
        // In debug mode, find the allocation in our tracking list
        std::lock_guard<std::mutex> lock(debugMutex);

        const std::uint8_t* bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);
        const std::uint8_t* memStart = reinterpret_cast<const std::uint8_t*>(memory);
        const MemorySize ptrOffset = bytePtr - memStart;

        for (const auto& alloc : debugAllocations) {
            if (alloc.offset == ptrOffset) {
                return alloc.size;
            }
        }
#endif

        // Can't determine size without debug info
        return 0;
    }

    std::size_t LinearAllocator::getAllocationCount() const {
        return allocationCount.load(std::memory_order_acquire);
    }

}
