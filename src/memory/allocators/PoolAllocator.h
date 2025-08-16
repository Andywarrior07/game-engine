//
// Created by Andres Guerrero on 12-08-25.
//

#pragma once
#include "../core/IAllocator.h"

namespace engine::memory {
    class PoolAllocator final : public IAllocator {
    public:
        PoolAllocator(MemorySize blockSize,
                      std::size_t blockCount,
                      MemorySize alignment = DEFAULT_ALIGNMENT,
                      const char* name = "PoolAllocator");
        ~PoolAllocator() override;

        PoolAllocator(const PoolAllocator&) = delete;
        PoolAllocator& operator=(const PoolAllocator&) = delete;
        PoolAllocator(PoolAllocator&& other) noexcept;
        PoolAllocator& operator=(PoolAllocator&& other) noexcept;

        void* allocate(MemorySize requestedSize, MemorySize requestedAlignment = DEFAULT_ALIGNMENT,
            AllocationFlags flags = AllocationFlags::NONE) override;
        void deallocate(void* ptr) override;
        MemorySize getCapacity() const override { return blockSize * blockCount; };
        MemorySize getUsedMemory() const override { return usedBlocks * blockSize; };
        void reset() override;
        bool owns(const void* ptr) const override;
        MemorySize getAllocationSize(const void* ptr) const override { return blockSize; };
        const char* getName() const override { return name; };

        std::size_t getFreeBlockCount() const { return blockCount - usedBlocks; };
        std::size_t getTotalBlockCount() const { return blockCount; }
        bool isFull() const { return usedBlocks >= blockCount; };
        static std::size_t defragment();

    private:
        // Node structure for free list
        struct FreeNode {
            FreeNode* next;
        };

        void* memory;
        MemorySize blockSize;
        std::size_t blockCount;
        MemorySize alignment;
        std::atomic<FreeNode*> freeList;
        std::atomic<std::size_t> usedBlocks;
        const char* name;

        void initializeFreeList();

#ifdef _DEBUG
        std::unordered_map<void*, bool> allocationMap;
        mutable std::mutex debugMutex;
#endif
    };
}
