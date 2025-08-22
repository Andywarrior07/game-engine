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
        MemorySize getCapacity() const override { return blockSize_ * blockCount_; };
        MemorySize getUsedMemory() const override { return usedBlocks_ * blockSize_; };
        void reset() override;
        bool owns(const void* ptr) const override;
        MemorySize getAllocationSize(const void* ptr) const override { return blockSize_; };
        const char* getName() const override { return name_; };

        std::size_t getFreeBlockCount() const { return blockCount_ - usedBlocks_; };
        std::size_t getTotalBlockCount() const { return blockCount_; }
        bool isFull() const { return usedBlocks_ >= blockCount_; };
        static std::size_t defragment();

    private:
        // Node structure for free list
        struct FreeNode {
            FreeNode* next;
        };

        void* memory_;
        MemorySize blockSize_;
        std::size_t blockCount_;
        MemorySize alignment_;
        std::atomic<FreeNode*> freeList_;
        std::atomic<std::size_t> usedBlocks_;
        const char* name_;

        void initializeFreeList();

#ifdef _DEBUG
        std::unordered_map<void*, bool> allocationMap_;
        mutable std::mutex debugMutex_;
#endif
    };
}
