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
        MemorySize getCapacity() const override { return capacity_; }
        MemorySize getUsedMemory() const override;
        void reset() override;
        bool owns(const void* ptr) const override;
        MemorySize getAllocationSize(const void* ptr) const override;
        const char* getName() const override { return name_; }

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

        void* memory_;
        MemorySize capacity_;
        std::atomic<MemorySize> current_;
        std::atomic<MemorySize> highWaterMark_;
        const char* name_;

#ifdef __DEBUG
        static constexpr std::uint32_t SENTINEL_VALUE = 0xDEADBEEF;
        std::atomic<std::uint32_t> allocationCount_{0};
#endif
    };
}
