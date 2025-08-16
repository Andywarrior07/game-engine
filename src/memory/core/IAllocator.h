//
// Created by Andres Guerrero on 12-08-25.
//

#pragma once

#include "MemoryStats.h"
#include "Types.h"

namespace engine::memory {
    class IAllocator {
    public:
        virtual ~IAllocator() = default;

        // Core interface methods
        virtual void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                               AllocationFlags flags = AllocationFlags::NONE) = 0;
        virtual void deallocate(void* ptr) = 0;

        // Query methods
        virtual MemorySize getCapacity() const = 0;
        virtual MemorySize getUsedMemory() const = 0;
        virtual bool owns(const void* ptr) const = 0;
        virtual const char* getName() const = 0;

        // Virtual methods with default implementation
        virtual void* reallocate(void* ptr, MemorySize newSize, MemorySize alignment = DEFAULT_ALIGNMENT);
        virtual MemorySize getFreeMemory() const;
        virtual void reset() {};
        virtual MemorySize getAllocationSize(const void* ptr) const = 0;
        virtual const MemoryStats& getStats() const { return stats; }

    protected:
        mutable MemoryStats stats;

        // Helper methods
        static void* alignPointer(void* ptr, MemorySize alignment);
        static MemorySize alignSize(MemorySize size, MemorySize alignment);
        static bool isPowerOfTwo(MemorySize value);

        void recordAllocation(MemorySize size, MemoryCategory category = MemoryCategory::GENERAL) const;
        void recordDeallocation(MemorySize size, MemoryCategory category = MemoryCategory::GENERAL) const;
    };
}
