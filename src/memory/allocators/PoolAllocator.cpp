//
// Created by Andres Guerrero on 12-08-25.
//

#include "PoolAllocator.h"

#include <cassert>
#include <iostream>
#include <ostream>

namespace engine::memory {
    PoolAllocator::PoolAllocator(const MemorySize blockSize,
                                 const std::size_t blockCount,
                                 const MemorySize alignment,
                                 const char* name)
        : memory(nullptr)
          , blockSize(alignSize(blockSize, alignment)) // Ensure block size is aligned
          , blockCount(blockCount)
          , alignment(alignment)
          , freeList(nullptr)
          , usedBlocks(0)
          , name(name) {
        // Validate parameters
        assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");
        assert(blockSize >= sizeof(FreeNode) && "Block size must be at least pointer size");
        assert(blockCount > 0 && "Block count must be greater than 0");

        // Calculate total memory needed
        const MemorySize totalSize = blockSize * blockCount;

        // Allocate aligned memory
        memory = std::aligned_alloc(alignment, totalSize);

        if (!memory) {
            throw std::bad_alloc();
        }

#ifdef _DEBUG
        std::memset(memory, 0xCD, totalSize);
#endif

        // Initialize free list
        initializeFreeList();

        // Initialize statistics
        stats.currentUsage = 0;
        stats.peakUsage = 0;
        stats.totalAllocated = 0;
        stats.allocationCount = 0;
    }

    PoolAllocator::~PoolAllocator() {
        if (!memory) return;

#ifdef _DEBUG
        if (usedBlocks.load() > 0) {
            std::cerr << "Warning: PoolAllocator '" << name
                << "' destroyed with " << usedBlocks.load()
                << " blocks still allocated!" << std::endl;

            // In debug mode, verify all allocations were freed
            std::lock_guard<std::mutex> lock(debugMutex);
            for (const auto& [ptr, allocated] : allocationMap) {
                if (allocated) {
                    std::cerr << "  Leaked block at: " << ptr << std::endl;
                }
            }
        }
#endif

        std::free(memory);
        memory = nullptr;
    }

    PoolAllocator::PoolAllocator(PoolAllocator&& other) noexcept
        : memory(other.memory)
          , blockSize(other.blockSize)
          , blockCount(other.blockCount)
          , alignment(other.alignment)
          , freeList(other.freeList.load())
          , usedBlocks(other.usedBlocks.load())
          , name(other.name) {
        other.memory = nullptr;
        other.blockSize = 0;
        other.blockCount = 0;
        other.freeList = nullptr;
        other.usedBlocks = 0;

#ifdef _DEBUG
        // Move debug allocation map
        std::lock_guard<std::mutex> lock(other.debugMutex);
        allocationMap = std::move(other.allocationMap);
#endif
    }

    PoolAllocator& PoolAllocator::operator=(PoolAllocator&& other) noexcept {
        if (this != &other) {
            // Free existing memory
            if (memory) {
                std::free(memory);
            }

            // Move from other
            memory = other.memory;
            blockSize = other.blockSize;
            blockCount = other.blockCount;
            alignment = other.alignment;
            freeList = other.freeList.load();
            usedBlocks = other.usedBlocks.load();
            name = other.name;

            // Reset other
            other.memory = nullptr;
            other.blockSize = 0;
            other.blockCount = 0;
            other.freeList = nullptr;
            other.usedBlocks = 0;

#ifdef _DEBUG
            // Move debug allocation map
            std::lock_guard<std::mutex> lock(other.debugMutex);
            allocationMap = std::move(other.allocationMap);
#endif
        }
        return *this;
    }

    void PoolAllocator::initializeFreeList() {
        auto* currentBlock = static_cast<std::uint8_t*>(memory);
        auto* firstNode = reinterpret_cast<FreeNode*>(currentBlock);
        FreeNode* currentNode = firstNode;

        for (std::size_t i = 0; i < blockCount - 1; i++) {
            auto* nextNode = reinterpret_cast<FreeNode*>(currentBlock + blockSize);
            currentNode->next = nextNode;
            currentNode = nextNode;
            currentBlock += blockSize;
        }

        currentNode->next = nullptr;

        freeList.store(firstNode, std::memory_order_release);

#ifdef _DEBUG
        // Initialize allocation tracking
        std::lock_guard<std::mutex> lock(debugMutex);
        allocationMap.clear();
        std::uint8_t* block = reinterpret_cast<std::uint8_t*>(memory);
        for (std::size_t i = 0; i < blockCount; ++i) {
            allocationMap[block] = false; // All blocks start as free
            block += blockSize;
        }
#endif
    }

    void* PoolAllocator::allocate(const MemorySize requestedSize, const MemorySize requestedAlignment, const AllocationFlags flags) {
        if (requestedSize > blockSize) {
            stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        if (requestedAlignment > alignment) {
            stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        FreeNode* head = freeList.load(std::memory_order_acquire);

        while (head != nullptr) {
            if (FreeNode* next = head->next; freeList.compare_exchange_weak(head, next,
                                                                            std::memory_order_acq_rel,
                                                                            std::memory_order_acquire)) {
                auto* block = reinterpret_cast<std::uint8_t*>(head);

                usedBlocks.fetch_add(1, std::memory_order_relaxed);
                recordAllocation(blockSize);

#ifdef _DEBUG
                // Track allocation
                {
                    std::lock_guard<std::mutex> lock(debugMutex_);
                    allocationMap_[block] = true;
                }
#endif

                if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
                    std::memset(block, 0, blockSize);
                }

#ifdef _DEBUG
                else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
                    std::memset(block, 0xAB, blockSize_); // Allocated pattern
                }
#endif

                return block;
            }

            head = freeList.load(std::memory_order_acquire);
        }

        stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
        return nullptr;
    }

    void PoolAllocator::deallocate(void* ptr) {
        if (!ptr) return;

        if (!owns(ptr)) {
            std::cerr << "Error: Attempted to deallocate pointer not owned by this pool!" << std::endl;
            assert(false && "Invalid deallocation");
        }

#ifdef _DEBUG
        // Check for double-free
        {
            std::lock_guard<std::mutex> lock(debugMutex_);
            auto it = allocationMap_.find(ptr);
            if (it == allocationMap_.end() || !it->second) {
                std::cerr << "Error: Double-free detected in PoolAllocator!" << std::endl;
                assert(false && "Double-free detected");
                return;
            }
            it->second = false; // Mark as free
        }

        // Fill with freed pattern
        std::memset(ptr, 0xDE, blockSize_);
#endif

        auto* node = static_cast<FreeNode*>(ptr);
        FreeNode* head = freeList.load(std::memory_order_acquire);

        do {
            node->next = head;
        }
        while (!freeList.compare_exchange_weak(head, node,
                                               std::memory_order_acq_rel,
                                               std::memory_order_acquire));

        usedBlocks.fetch_sub(1, std::memory_order_relaxed);
        recordDeallocation(blockSize);
    }

    void PoolAllocator::reset() {
        usedBlocks.store(0, std::memory_order_release);

#ifdef _DEBUG
        // Clear memory with uninitialized pattern
        std::memset(memory_, 0xCD, blockSize_ * blockCount_);
#endif

        initializeFreeList();

        stats.currentUsage.store(0, std::memory_order_release);
    }

    bool PoolAllocator::owns(const void* ptr) const {
        if (!ptr || !memory) return false;

        const auto bytePtr = static_cast<const std::uint8_t*>(ptr);
        const auto memStart = static_cast<const std::uint8_t*>(memory);

        if (const std::uint8_t* memEnd = memStart + (blockSize * blockCount); bytePtr < memStart || bytePtr >= memEnd) {
            return false;
        }

        const std::ptrdiff_t offset = bytePtr - memStart;

        return (offset % blockSize) == 0;
    }

    std::size_t PoolAllocator::defragment() {
        // Pool allocator doesn't fragment in the traditional sense
        // All blocks are the same size, so no defragmentation needed
        // This could be used to sort the free list for better cache locality

        // For now, just return 0 (no blocks defragmented)
        return 0;
    }
}
