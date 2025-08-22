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
        : memory_(nullptr)
          , blockSize_(alignSize(blockSize, alignment)) // Ensure block size is aligned
          , blockCount_(blockCount)
          , alignment_(alignment)
          , freeList_(nullptr)
          , usedBlocks_(0)
          , name_(name) {
        // Validate parameters
        assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");
        assert(blockSize >= sizeof(FreeNode) && "Block size must be at least pointer size");
        assert(blockCount > 0 && "Block count must be greater than 0");

        // Calculate total memory needed
        const MemorySize totalSize = blockSize * blockCount;

        // Allocate aligned memory
        memory_ = std::aligned_alloc(alignment, totalSize);

        if (!memory_) {
            std::cerr << "[PoolAllocator] Failed to allocate: " << name_ << std::endl;
            throw std::bad_alloc();
        }


#ifdef _DEBUG
        std::memset(memory_, 0xCD, totalSize);
#endif
        // Initialize free list
        initializeFreeList();
        // Initialize statistics
        stats_.currentUsage = 0;
        stats_.peakUsage = 0;
        stats_.totalAllocated = 0;
        stats_.allocationCount = 0;
    }

    PoolAllocator::~PoolAllocator() {
        if (!memory_) return;

#ifdef _DEBUG
        if (usedBlocks_.load() > 0) {
            std::cerr << "Warning: PoolAllocator '" << name_
                << "' destroyed with " << usedBlocks_.load()
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

        std::free(memory_);
        memory_ = nullptr;
    }

    PoolAllocator::PoolAllocator(PoolAllocator&& other) noexcept
        : memory_(other.memory_)
          , blockSize_(other.blockSize_)
          , blockCount_(other.blockCount_)
          , alignment_(other.alignment_)
          , freeList_(other.freeList_.load())
          , usedBlocks_(other.usedBlocks_.load())
          , name_(other.name_) {
        other.memory_ = nullptr;
        other.blockSize_ = 0;
        other.blockCount_ = 0;
        other.freeList_ = nullptr;
        other.usedBlocks_ = 0;

#ifdef _DEBUG
        // Move debug allocation map
        std::lock_guard<std::mutex> lock(other.debugMutex);
        allocationMap = std::move(other.allocationMap);
#endif
    }

    PoolAllocator& PoolAllocator::operator=(PoolAllocator&& other) noexcept {
        if (this != &other) {
            // Free existing memory
            if (memory_) {
                std::free(memory_);
            }

            // Move from other
            memory_ = other.memory_;
            blockSize_ = other.blockSize_;
            blockCount_ = other.blockCount_;
            alignment_ = other.alignment_;
            freeList_ = other.freeList_.load();
            usedBlocks_ = other.usedBlocks_.load();
            name_ = other.name_;

            // Reset other
            other.memory_ = nullptr;
            other.blockSize_ = 0;
            other.blockCount_ = 0;
            other.freeList_ = nullptr;
            other.usedBlocks_ = 0;

#ifdef _DEBUG
            // Move debug allocation map
            std::lock_guard<std::mutex> lock(other.debugMutex);
            allocationMap = std::move(other.allocationMap);
#endif
        }
        return *this;
    }

    void PoolAllocator::initializeFreeList() {
        auto* currentBlock = static_cast<std::uint8_t*>(memory_);
        auto* firstNode = reinterpret_cast<FreeNode*>(currentBlock);
        FreeNode* currentNode = firstNode;

        for (std::size_t i = 0; i < blockCount_ - 1; i++) {
            auto* nextNode = reinterpret_cast<FreeNode*>(currentBlock + blockSize_);
            currentNode->next = nextNode;
            currentNode = nextNode;
            currentBlock += blockSize_;
        }

        currentNode->next = nullptr;

        freeList_.store(firstNode, std::memory_order_release);

#ifdef _DEBUG
        // Initialize allocation tracking
        std::lock_guard<std::mutex> lock(debugMutex);
        allocationMap.clear();
        std::uint8_t* block = reinterpret_cast<std::uint8_t*>(memory_);
        for (std::size_t i = 0; i < blockCount_; ++i) {
            allocationMap[block] = false; // All blocks start as free
            block += blockSize_;
        }
#endif
    }

    void* PoolAllocator::allocate(const MemorySize requestedSize, const MemorySize requestedAlignment, const AllocationFlags flags) {
        if (requestedSize > blockSize_) {
            stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        if (requestedAlignment > alignment_) {
            stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        FreeNode* head = freeList_.load(std::memory_order_acquire);

        while (head != nullptr) {
            if (FreeNode* next = head->next; freeList_.compare_exchange_weak(head, next,
                                                                            std::memory_order_acq_rel,
                                                                            std::memory_order_acquire)) {
                auto* block = reinterpret_cast<std::uint8_t*>(head);

                usedBlocks_.fetch_add(1, std::memory_order_relaxed);
                recordAllocation(blockSize_);

#ifdef _DEBUG
                // Track allocation
                {
                    std::lock_guard<std::mutex> lock(debugMutex_);
                    allocationMap_[block] = true;
                }
#endif

                if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
                    std::memset(block, 0, blockSize_);
                }

#ifdef _DEBUG
                else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
                    std::memset(block, 0xAB, blockSize_); // Allocated pattern
                }
#endif

                return block;
            }

            head = freeList_.load(std::memory_order_acquire);
        }

        stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
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
        FreeNode* head = freeList_.load(std::memory_order_acquire);

        do {
            node->next = head;
        }
        while (!freeList_.compare_exchange_weak(head, node,
                                               std::memory_order_acq_rel,
                                               std::memory_order_acquire));

        usedBlocks_.fetch_sub(1, std::memory_order_relaxed);
        recordDeallocation(blockSize_);
    }

    void PoolAllocator::reset() {
        usedBlocks_.store(0, std::memory_order_release);

#ifdef _DEBUG
        // Clear memory with uninitialized pattern
        std::memset(memory_, 0xCD, blockSize_ * blockCount_);
#endif

        initializeFreeList();

        stats_.currentUsage.store(0, std::memory_order_release);
    }

    bool PoolAllocator::owns(const void* ptr) const {
        if (!ptr || !memory_) return false;

        const auto bytePtr = static_cast<const std::uint8_t*>(ptr);
        const auto memStart = static_cast<const std::uint8_t*>(memory_);

        if (const std::uint8_t* memEnd = memStart + (blockSize_ * blockCount_); bytePtr < memStart || bytePtr >= memEnd) {
            return false;
        }

        const std::ptrdiff_t offset = bytePtr - memStart;

        return (offset % blockSize_) == 0;
    }

    std::size_t PoolAllocator::defragment() {
        // Pool allocator doesn't fragment in the traditional sense
        // All blocks are the same size, so no defragmentation needed
        // This could be used to sort the free list for better cache locality

        // For now, just return 0 (no blocks defragmented)
        return 0;
    }
}
