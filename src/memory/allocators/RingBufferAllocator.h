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
        MemorySize getCapacity() const override { return capacity; }
        MemorySize getUsedMemory() const override;
        void reset() override;
        bool owns(const void* ptr) const override;
        MemorySize getAllocationSize(const void* ptr) const override;
        const char* getName() const override { return name; }

        std::uint64_t createFence();
        void waitForFence(std::uint64_t fence);
        bool canAllocateWithoutWrap(MemorySize size) const;

    private:
        struct AllocationHeader {
            MemorySize size;
            std::uint64_t fence;
#ifdef _DEBUG
            std::uint32_t magic; // Magic number for validation
#endif
        };

        void* memory;
        MemorySize capacity;
        std::atomic<MemorySize> head;
        std::atomic<MemorySize> tail;
        std::atomic<std::uint64_t> fenceCounter;
        const char* name;

        std::mutex allocationMutex;

#ifdef _DEBUG
        static constexpr std::uint32_t MAGIC_NUMBER = 0xFEEDFACE;
#endif
    };
}
