//
// Created by Andres Guerrero on 12-08-25.
//

#include "StackAllocator.h"

#include <cassert>
#include <iostream>

namespace engine::memory {
    StackAllocator::StackAllocator(MemorySize capacity, const char* name)
        : memory_(nullptr)
          , capacity_(capacity)
          , current_(0)
          , highWaterMark_(0)
          , name_(name) {
        memory_ = std::aligned_alloc(CACHE_LINE_SIZE, capacity);

        if (!memory_) {
            std::cerr << "[StackAllocator] Failed to allocate "
                << (capacity / (1024.0 * 1024.0)) << " MB for " << name_ << std::endl;
            throw std::bad_alloc();
        }

#ifdef _DEBUG
        std::memset(memory_, 0xCD, capacity);
#endif

        stats_.currentUsage = 0;
        stats_.peakUsage = 0;
        stats_.totalAllocated = 0;
        stats_.allocationCount = 0;
    }

    StackAllocator::~StackAllocator() {
        if (!memory_) return;

#ifdef _DEBUG
        if (current_.load() > 0) {
            std::cerr << "Warning: StackAllocator '" << name_
                << "' destroyed with " << current_.load()
                << " bytes still allocated!" << std::endl;
        }
#endif

        std::free(memory_);
        memory_ = nullptr;
    }

    StackAllocator::StackAllocator(StackAllocator&& other) noexcept
        : memory_(other.memory_)
          , capacity_(other.capacity_)
          , current_(other.current_.load())
          , highWaterMark_(other.highWaterMark_.load())
          , name_(other.name_) {
        other.memory_ = nullptr;
        other.capacity_ = 0;
        other.current_ = 0;
        other.highWaterMark_ = 0;
    }

    StackAllocator& StackAllocator::operator=(StackAllocator&& other) noexcept {
        if (this == &other) return *this;

        if (memory_) {
            std::free(memory_);
        }

        memory_ = other.memory_;
        capacity_ = other.capacity_;
        current_ = other.current_.load();
        highWaterMark_ = other.highWaterMark_.load();
        name_ = other.name_;

        // reset other
        other.memory_ = nullptr;
        other.capacity_ = 0;
        other.current_ = 0;
        other.highWaterMark_ = 0;

        return *this;
    }

    void* StackAllocator::allocate(const MemorySize size, const MemorySize alignment, const AllocationFlags flags) {
        assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");

        // Calculate total size including header
        constexpr MemorySize headerSize = sizeof(AllocationHeader);
        const MemorySize totalSize = size + headerSize;

        // Get current position and calculate aligned address
        MemorySize currentPos = current_.load(std::memory_order_acquire);
        void* currentAddress = static_cast<std::uint8_t*>(memory_) + currentPos;
        void* alignedAddress = alignPointer(currentAddress, alignment);

        // Calculate adjustment for alignment
        const MemorySize adjustment = static_cast<std::uint8_t*>(alignedAddress) - static_cast<std::uint8_t*>(
            currentAddress);

        // Calculate a new position after allocation
        MemorySize newPos = currentPos + adjustment + totalSize;

        // Check if allocation fits
        if (newPos > capacity_) {
            stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        // Atomic update of current position
        MemorySize expected = currentPos;
        while (!current_.compare_exchange_weak(expected, newPos, std::memory_order_acq_rel,
                                               std::memory_order_acquire)) {
            // Another thread allocated, recalculate
            currentPos = expected;
            currentAddress = static_cast<std::uint8_t*>(memory_) + currentPos;
            alignedAddress = alignPointer(currentAddress, alignment);
            const MemorySize newAdjustment = static_cast<std::uint8_t*>(alignedAddress) -
                static_cast<std::uint8_t*>(currentAddress);
            const MemorySize newNewPos = currentPos + newAdjustment + totalSize;

            if (newNewPos > capacity_) {
                stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
                return nullptr;
            }

            newPos = newNewPos;
        }

        // Update high water mark
        MemorySize currentHigh = highWaterMark_.load(std::memory_order_acquire);
        while (newPos > currentHigh &&
            !highWaterMark_.compare_exchange_weak(currentHigh, newPos,
                                                  std::memory_order_acq_rel,
                                                  std::memory_order_acquire)) {
            // Keep trying
            /*
             * Esto es por si llega a haber un pequeño fallo en compare_exchange_weak (que es el comportamiento normal)
             * se reintenta por el bucle
             */
        }

        // Write allocation header
        auto* header = static_cast<AllocationHeader*>(alignedAddress);
        header->size = size;
        header->adjustment = adjustment;
#ifdef _DEBUG
        header->sentinel = SENTINEL_VALUE;
        allocationCount_.fetch_add(1, std::memory_order_relaxed);
#endif

        // Get user pointer (after header)
        void* userPtr = static_cast<std::uint8_t*>(alignedAddress) + headerSize;

        // Handle allocation flags
        if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
            std::memset(userPtr, 0, size);
        }
#ifdef _DEBUG
      else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
            std::memset(userPtr, 0xAB, size); // Allocated memory pattern
        }
#endif

        // Update statistics
        recordAllocation(size + adjustment + headerSize);

        return userPtr;
    }

    void StackAllocator::deallocate(void* ptr) {
        if (!ptr) return;

        // Stack allocator only supports freeing from the top
        // Check if this is the most recent allocation
#ifdef _DEBUG
        // In debug mode, verify the allocation
        AllocationHeader* header = reinterpret_cast<AllocationHeader*>(
            reinterpret_cast<std::uint8_t*>(ptr) - sizeof(AllocationHeader)
        );

        if (header->sentinel != SENTINEL_VALUE) {
            std::cerr << "Error: Corrupted allocation header in StackAllocator!" << std::endl;
            assert(false && "Memory corruption detected");
        }

        // Calculate if this is the top allocation
        void* headerAddress = header;
        MemorySize allocStart = reinterpret_cast<std::uint8_t*>(headerAddress) -
            reinterpret_cast<std::uint8_t*>(memory_);
        MemorySize allocEnd = allocStart + header->adjustment + sizeof(AllocationHeader) + header->size;

        if (allocEnd == current_.load(std::memory_order_acquire)) {
            // This is the top allocation, we can free it
            current_.store(allocStart, std::memory_order_release);
            recordDeallocation(header->size + header->adjustment + sizeof(AllocationHeader));
            allocationCount_.fetch_sub(1, std::memory_order_relaxed);

            // Fill with freed pattern
            std::memset(headerAddress, 0xDE, allocEnd - allocStart);
        }
        else {
            // Not the top allocation - can't free individual allocations in stack
            std::cerr << "Warning: Attempted to free non-top allocation in StackAllocator. "
                << "Use freeToMarker() or reset() instead." << std::endl;
        }
#else
        // In release mode, stack doesn't support individual deallocation
        // This is a no-op to maintain the stack property
#endif
    }

    MemorySize StackAllocator::getUsedMemory() const {
        return current_.load(std::memory_order_acquire);
    }

    void StackAllocator::reset() {
#ifdef _DEBUG
        // Fill freed memory with pattern
        MemorySize used = current_.load(std::memory_order_acquire);
        if (used > 0) {
            std::memset(memory_, 0xCD, used); // Reset to uninitialized pattern
        }
        allocationCount_.store(0, std::memory_order_release);
#endif
        current_.store(0, std::memory_order_release);

        stats_.currentUsage.store(0, std::memory_order_release);
    }

    StackAllocator::Marker StackAllocator::getMarker() const {
        // Return the current stack position as a marker
        // This marker can be used later to restore the stack to this exact position
        return current_.load(std::memory_order_acquire);
    }

    void StackAllocator::freeToMarker(const Marker marker) {
        const MemorySize currentPos = current_.load(std::memory_order_acquire);

        if (marker > currentPos) {
            std::cerr << "Error: Invalid marker in StackAllocator::freeToMarker!" << std::endl;
            return;
        }

#ifdef _DEBUG
        // Fill freed memory with pattern
        MemorySize freedSize = currentPos - marker;
        if (freedSize > 0) {
            void* freedStart = reinterpret_cast<std::uint8_t*>(memory_) + marker;
            std::memset(freedStart, 0xDE, freedSize);
        }
#endif

        current_.store(marker, std::memory_order_release);

        recordDeallocation(currentPos - marker);
    }

    MemorySize StackAllocator::getHighWaterMark() const {
        // Return the maximum amount of memory that has ever been used
        // This is useful for profiling to know the peak memory usage
        return highWaterMark_.load(std::memory_order_acquire);
    }

    bool StackAllocator::owns(const void* ptr) const {
        if (!ptr || !memory_) return false;

        auto* bytePtr = static_cast<const std::uint8_t*>(ptr);
        auto* memStart = static_cast<const std::uint8_t*>(memory_);
        auto* memEnd = memStart + capacity_;

        return bytePtr >= memStart && bytePtr < memEnd;
    }

    MemorySize StackAllocator::getAllocationSize(const void* ptr) const {
        if (!owns(ptr)) return 0;

#ifdef _DEBUG
        // In debug mode, we can get the actual size from the header
        const AllocationHeader* header = reinterpret_cast<const AllocationHeader*>(
            reinterpret_cast<const std::uint8_t*>(ptr) - sizeof(AllocationHeader)
        );

        if (header->sentinel == SENTINEL_VALUE) {
            return header->size;
        }
#endif

        // Without debug info, we can't determine individual allocation sizes
        return 0;
    }
}
