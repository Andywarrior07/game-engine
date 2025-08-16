//
// Created by Andres Guerrero on 12-08-25.
//

#pragma once
#include "../core/IAllocator.h"

namespace engine::memory {
    class StackAllocator final : public IAllocator {
    public:
        using Marker = MemorySize;

        explicit StackAllocator(MemorySize capacity, const char* name = "StackAllocator");
        ~StackAllocator() override;

        // No copy, allow move
        StackAllocator(const StackAllocator&) = delete;
        StackAllocator& operator=(const StackAllocator&) = delete;
        StackAllocator(StackAllocator&& other) noexcept;
        StackAllocator& operator=(StackAllocator&& other) noexcept;

        // IAllocator interface
        void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                       AllocationFlags flags = AllocationFlags::NONE) override;
        void deallocate(void* ptr) override;
        MemorySize getCapacity() const override { return capacity; }
        MemorySize getUsedMemory() const override;
        void reset() override;
        bool owns(const void* ptr) const override;
        MemorySize getAllocationSize(const void* ptr) const override;
        const char* getName() const override { return name; }

        // Stack-specific methods
        Marker getMarker() const;
        void freeToMarker(Marker marker);
        MemorySize getHighWaterMark() const;

    private:
        struct AllocationHeader {
            MemorySize size;
            MemorySize adjustment;
#ifdef _DEBUF
            std::uint32_t sentinel;
#endif
        };

        void* memory;
        MemorySize capacity;
        std::atomic<MemorySize> current;
        std::atomic<MemorySize> highWaterMark;
        const char* name;

#ifdef __DEBUG
        static constexpr std::uint32_t SENTINEL_VALUE = 0xDEADBEEF;
        std::atomic<std::uint32_t> allocationCount{0};
#endif
    };
}
