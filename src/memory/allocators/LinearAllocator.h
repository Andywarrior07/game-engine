//
// Created by Andres Guerrero on 14-08-25.
//

#pragma once
#include "../core/IAllocator.h"

namespace engine::memory {
    class LinearAllocator final : public IAllocator {
    public:
        explicit LinearAllocator(MemorySize capacity, const char* name = "LinearAllocator");
        ~LinearAllocator() override;

        LinearAllocator(const LinearAllocator&) = delete;
        LinearAllocator& operator=(const LinearAllocator&) = delete;

        LinearAllocator(LinearAllocator&& other) noexcept;
        LinearAllocator& operator=(LinearAllocator&& other) noexcept;

        void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                       AllocationFlags flags = AllocationFlags::NONE) override;
        void deallocate(void* ptr) override;
        MemorySize getCapacity() const override { return capacity; }
        MemorySize getUsedMemory() const override;
        void reset() override;
        bool owns(const void* ptr) const override;
        MemorySize getAllocationSize(const void* ptr) const override;
        const char* getName() const override { return name; }

        std::size_t getAllocationCount() const;

    private:
        void* memory;
        MemorySize capacity;
        std::atomic<MemorySize> current;
        std::atomic<std::size_t> allocationCount;
        const char* name;

#ifdef _DEBUG
        // Track allocations for debugging
        struct DebugAllocation {
            MemorySize offset;
            MemorySize size;
        };
        std::vector<DebugAllocation> debugAllocations_;
        mutable std::mutex debugMutex_;
#endif
    };
}
