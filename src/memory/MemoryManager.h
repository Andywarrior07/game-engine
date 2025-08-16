//
// Created by Andres Guerrero on 11-08-25.
//

// #pragma once
//
// #include <cstddef>          // For std::size_t, std::ptrdiff_t
// #include <cstdint>          // For fixed-width integer types
// #include <memory>           // For std::unique_ptr, std::align
// #include <atomic>           // For lock-free counters
// #include <mutex>            // For thread synchronization
// #include <shared_mutex>     // For reader-writer locks
// #include <vector>           // For dynamic arrays
// #include <unordered_map>    // For allocation tracking
// #include <chrono>           // For timing and profiling
// #include <string>           // For debug names
// #include <cassert>          // For debug assertions
// #include <new>              // For placement new
// #include <algorithm>        // For std::max, std::min
// #include <type_traits>      // For type traits and SFINAE
// #include <functional>       // For callbacks
// #include <array>            // For fixed-size arrays
// #include <optional>         // For optional values
//
// #include "allocators/StackAllocator.h"
// #include "allocators/PoolAllocator.h"
// #include "core/Types.h"
// #include "core/IAllocator.h"
// #include "manager/MemoryConfig.h"

// namespace engine::memory {
    // ========================================================================
    // FORWARD DECLARATIONS AND TYPE ALIASES
    // ========================================================================

    // class IAllocator;
    // class MemoryManager;
    // class StackAllocator;
    // class PoolAllocator;
    // class LinearAllocator;
    // class RingBufferAllocator;
    // class HeapAllocator;
    // class ScopedAllocator;

    // Memory statistics for profiling
    // struct MemoryStats {
    //     std::atomic<MemorySize> totalAllocated{0}; // Total bytes allocated
    //     std::atomic<MemorySize> totalFreed{0}; // Total bytes freed
    //     std::atomic<MemorySize> currentUsage{0}; // Current bytes in use
    //     std::atomic<MemorySize> peakUsage{0}; // Peak memory usage
    //     std::atomic<std::uint64_t> allocationCount{0}; // Number of allocations
    //     std::atomic<std::uint64_t> freeCount{0}; // Number of deallocations
    //     std::atomic<std::uint64_t> failedAllocations{0}; // Failed allocation attempts
    //
    //     // Per-category statistics
    //     std::array<std::atomic<MemorySize>, static_cast<std::size_t>(MemoryCategory::COUNT)> categoryUsage{};
    //
    //     // Timing statistics
    //     std::atomic<std::uint64_t> totalAllocationTime{0}; // Total time spent allocating (microseconds)
    //     std::atomic<std::uint64_t> totalFreeTime{0}; // Total time spent freeing (microseconds)
    // };

    // Allocation metadata for tracking
    struct AllocationInfo {
        void* address; // Memory address
        MemorySize size; // Allocation size
        MemorySize alignment; // Alignment requirement
        MemoryCategory category; // Usage category
        AllocationFlags flags; // Allocation flags
        std::chrono::steady_clock::time_point timestamp; // When allocated
        std::string debugName; // Optional debug name
        AllocationID id; // Unique allocation ID

#ifdef _DEBUG
        std::string filename; // Source file (debug only)
        std::uint32_t line; // Source line (debug only)
        std::string function; // Function name (debug only)
#endif
    };

    // ========================================================================
    // BASE ALLOCATOR INTERFACE
    // ========================================================================

    /**
     * @brief Abstract base class for all memory allocators
     *
     * Defines the common interface that all allocators must implement.
     * Supports aligned allocations, debug tracking, and statistics.
     */
    // class IAllocator {
    // public:
    //     virtual ~IAllocator() = default;
    //
    //     /**
    //      * @brief Allocate memory with specified size and alignment
    //      * @param size Number of bytes to allocate
    //      * @param alignment Memory alignment requirement (must be power of 2)
    //      * @param flags Optional allocation flags
    //      * @return Pointer to allocated memory, or nullptr on failure
    //      */
    //     virtual void* allocate(MemorySize size,
    //                            MemorySize alignment = DEFAULT_ALIGNMENT,
    //                            AllocationFlags flags = AllocationFlags::NONE) = 0;
    //
    //     /**
    //      * @brief Deallocate previously allocated memory
    //      * @param ptr Pointer to memory to free
    //      */
    //     virtual void deallocate(void* ptr) = 0;
    //
    //     /**
    //      * @brief Reallocate memory to a new size
    //      * @param ptr Original memory pointer
    //      * @param newSize New size in bytes
    //      * @param alignment Alignment requirement
    //      * @return Pointer to reallocated memory, or nullptr on failure
    //      */
    //     virtual void* reallocate(void* ptr, MemorySize newSize,
    //                              MemorySize alignment = DEFAULT_ALIGNMENT) {
    //         // Default implementation: allocate new, copy, free old
    //         if (!ptr) {
    //             return allocate(newSize, alignment);
    //         }
    //
    //         if (newSize == 0) {
    //             deallocate(ptr);
    //             return nullptr;
    //         }
    //
    //         void* newPtr = allocate(newSize, alignment);
    //         if (newPtr && ptr) {
    //             // Get original size if tracking is enabled
    //             MemorySize oldSize = getAllocationSize(ptr);
    //             std::memcpy(newPtr, ptr, std::min(oldSize, newSize));
    //             deallocate(ptr);
    //         }
    //
    //         return newPtr;
    //     }
    //
    //     /**
    //      * @brief Get the total memory capacity of this allocator
    //      * @return Total bytes available
    //      */
    //     virtual MemorySize getCapacity() const = 0;
    //
    //     /**
    //      * @brief Get current memory usage
    //      * @return Bytes currently allocated
    //      */
    //     virtual MemorySize getUsedMemory() const = 0;
    //
    //     /**
    //      * @brief Get available memory
    //      * @return Bytes available for allocation
    //      */
    //     virtual MemorySize getFreeMemory() const {
    //         return getCapacity() - getUsedMemory();
    //     }
    //
    //     /**
    //      * @brief Reset allocator to initial state (if supported)
    //      */
    //     virtual void reset() {
    //     }
    //
    //     /**
    //      * @brief Check if allocator owns a given pointer
    //      * @param ptr Pointer to check
    //      * @return True if this allocator allocated the pointer
    //      */
    //     virtual bool owns(const void* ptr) const = 0;
    //
    //     /**
    //      * @brief Get allocation size for a given pointer
    //      * @param ptr Allocated pointer
    //      * @return Size of allocation, or 0 if not found
    //      */
    //     virtual MemorySize getAllocationSize(const void* ptr) const = 0;
    //
    //     /**
    //      * @brief Get allocator name for debugging
    //      * @return Human-readable allocator name
    //      */
    //     virtual const char* getName() const = 0;
    //
    //     /**
    //      * @brief Get memory statistics
    //      * @return Current memory statistics
    //      */
    //     virtual const MemoryStats& getStats() const { return stats; }
    //
    // protected:
    //     mutable MemoryStats stats; // Memory usage statistics
    //
    //     // Helper to align memory addresses
    //     static void* alignPointer(void* ptr, MemorySize alignment) {
    //         const uintptr_t mask = alignment - 1;
    //         const uintptr_t misalignment = reinterpret_cast<uintptr_t>(ptr) & mask;
    //         const MemorySize adjustment = misalignment ? (alignment - misalignment) : 0;
    //         return reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(ptr) + adjustment);
    //     }
    //
    //     // Helper to calculate aligned size
    //     static MemorySize alignSize(MemorySize size, MemorySize alignment) {
    //         const MemorySize mask = alignment - 1;
    //         return (size + mask) & ~mask;
    //     }
    //
    //     // Check if value is power of 2
    //     // static bool isPowerOfTwo(MemorySize value) {
    //     //     return value && !(value & (value - 1));
    //     // }
    //
    //     // Update statistics on allocation
    //     void recordAllocation(MemorySize size, MemoryCategory category = MemoryCategory::GENERAL) {
    //         stats.totalAllocated.fetch_add(size, std::memory_order_relaxed);
    //         stats.currentUsage.fetch_add(size, std::memory_order_relaxed);
    //         stats.allocationCount.fetch_add(1, std::memory_order_relaxed);
    //         stats.categoryUsage[static_cast<std::size_t>(category)].fetch_add(size, std::memory_order_relaxed);
    //
    //         // Update peak usage
    //         MemorySize current = stats.currentUsage.load(std::memory_order_relaxed);
    //         MemorySize peak = stats.peakUsage.load(std::memory_order_relaxed);
    //         while (current > peak && !stats.peakUsage.compare_exchange_weak(peak, current)) {
    //             // Keep trying until we update peak or current is no longer greater
    //         }
    //     }
    //
    //     // Update statistics on deallocation
    //     void recordDeallocation(MemorySize size, MemoryCategory category = MemoryCategory::GENERAL) {
    //         stats.totalFreed.fetch_add(size, std::memory_order_relaxed);
    //         stats.currentUsage.fetch_sub(size, std::memory_order_relaxed);
    //         stats.freeCount.fetch_add(1, std::memory_order_relaxed);
    //         stats.categoryUsage[static_cast<std::size_t>(category)].fetch_sub(size, std::memory_order_relaxed);
    //     }
    // };

    // ========================================================================
    // STACK ALLOCATOR (LIFO)
    // ========================================================================

    /**
     * @brief Stack-based allocator for temporary allocations
     *
     * Extremely fast allocation and deallocation in LIFO order.
     * Perfect for temporary per-frame allocations that follow stack semantics.
     *
     * Performance characteristics:
     * - Allocation: O(1) - just increment pointer
     * - Deallocation: O(1) - just decrement pointer
     * - No fragmentation when used properly
     * - Excellent cache locality
     */
//     class StackAllocator : public IAllocator {
//     public:
//         /**
//          * @brief Marker for stack position restoration
//          */
//         using Marker = MemorySize;
//
//         /**
//          * @brief Constructor with specified capacity
//          * @param capacity Total stack size in bytes
//          * @param name Optional debug name
//          */
//         explicit StackAllocator(MemorySize capacity, const char* name = "StackAllocator");
//
//         /**
//          * @brief Destructor - frees backing memory
//          */
//         ~StackAllocator() override;
//
//         // Delete copy operations
//         StackAllocator(const StackAllocator&) = delete;
//         StackAllocator& operator=(const StackAllocator&) = delete;
//
//         // Allow move operations
//         StackAllocator(StackAllocator&& other) noexcept;
//         StackAllocator& operator=(StackAllocator&& other) noexcept;
//
//         // IAllocator interface implementation
//         void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
//                        AllocationFlags flags = AllocationFlags::NONE) override;
//         void deallocate(void* ptr) override;
//         MemorySize getCapacity() const override { return capacity; }
//         MemorySize getUsedMemory() const override { return current.load(std::memory_order_acquire); }
//         void reset() override;
//         bool owns(const void* ptr) const override;
//         MemorySize getAllocationSize(const void* ptr) const override;
//         const char* getName() const override { return name; }
//
//         /**
//          * @brief Get current stack position marker
//          * @return Marker representing current stack state
//          */
//         Marker getMarker() const { return current.load(std::memory_order_acquire); }
//
//         /**
//          * @brief Restore stack to a previous marker position
//          * @param marker Previously obtained marker
//          */
//         void freeToMarker(Marker marker);
//
//         /**
//          * @brief Get high water mark (maximum usage)
//          * @return Maximum bytes ever used
//          */
//         MemorySize getHighWaterMark() const { return highWaterMark.load(std::memory_order_acquire); }
//
//     private:
//         // Header stored before each allocation for tracking
//         struct AllocationHeader {
//             MemorySize size; // Size of this allocation
//             MemorySize adjustment; // Bytes added for alignment
// #ifdef _DEBUG
//             std::uint32_t sentinel; // Guard value for corruption detection
// #endif
//         };
//
//         void* memory; // Base memory pointer
//         MemorySize capacity; // Total capacity
//         std::atomic<MemorySize> current; // Current stack pointer
//         std::atomic<MemorySize> highWaterMark; // Maximum usage tracking
//         const char* name; // Debug name
//
// #ifdef _DEBUG
//         static constexpr std::uint32_t SENTINEL_VALUE = 0xDEADBEEF;
//         std::atomic<std::uint32_t> allocationCount_{0}; // Track allocation count for debugging
// #endif
//     };

    // ========================================================================
    // POOL ALLOCATOR (FIXED SIZE BLOCKS)
    // ========================================================================

    /**
     * @brief Pool allocator for fixed-size allocations
     *
     * Optimized for allocating many objects of the same size.
     * Zero fragmentation, constant-time allocation and deallocation.
     * Perfect for game objects, particles, bullets, etc.
     *
     * Performance characteristics:
     * - Allocation: O(1) - pop from free list
     * - Deallocation: O(1) - push to free list
     * - No fragmentation
     * - Predictable memory layout
     */
//     class PoolAllocator : public IAllocator {
//     public:
//         /**
//          * @brief Constructor for pool allocator
//          * @param blockSize Size of each block in bytes
//          * @param blockCount Number of blocks in the pool
//          * @param alignment Memory alignment for blocks
//          * @param name Optional debug name
//          */
//         PoolAllocator(MemorySize blockSize,
//                       std::size_t blockCount,
//                       MemorySize alignment = DEFAULT_ALIGNMENT,
//                       const char* name = "PoolAllocator");
//
//         /**
//          * @brief Destructor - frees pool memory
//          */
//         ~PoolAllocator() override;
//
//         // Delete copy operations
//         PoolAllocator(const PoolAllocator&) = delete;
//         PoolAllocator& operator=(const PoolAllocator&) = delete;
//
//         // Allow move operations
//         PoolAllocator(PoolAllocator&& other) noexcept;
//         PoolAllocator& operator=(PoolAllocator&& other) noexcept;
//
//         // IAllocator interface implementation
//         void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
//                        AllocationFlags flags = AllocationFlags::NONE) override;
//         void deallocate(void* ptr) override;
//         MemorySize getCapacity() const override { return blockSize_ * blockCount_; }
//         MemorySize getUsedMemory() const override { return usedBlocks_ * blockSize_; }
//         void reset() override;
//         bool owns(const void* ptr) const override;
//         MemorySize getAllocationSize(const void* ptr) const override { return blockSize_; }
//         const char* getName() const override { return name_; }
//
//         /**
//          * @brief Get number of free blocks available
//          * @return Count of free blocks
//          */
//         std::size_t getFreeBlockCount() const { return blockCount_ - usedBlocks_; }
//
//         /**
//          * @brief Get total number of blocks
//          * @return Total block count
//          */
//         std::size_t getTotalBlockCount() const { return blockCount_; }
//
//         /**
//          * @brief Check if pool is full
//          * @return True if no free blocks available
//          */
//         bool isFull() const { return usedBlocks_ >= blockCount_; }
//
//         /**
//          * @brief Defragment the pool (if supported)
//          * @return Number of blocks defragmented
//          */
//         std::size_t defragment();
//
//     private:
//         // Node structure for free list
//         struct FreeNode {
//             FreeNode* next;
//         };
//
//         void* memory_; // Base memory pointer
//         MemorySize blockSize_; // Size of each block
//         std::size_t blockCount_; // Total number of blocks
//         MemorySize alignment_; // Block alignment
//         std::atomic<FreeNode*> freeList_; // Head of free list (lock-free)
//         std::atomic<std::size_t> usedBlocks_; // Number of blocks in use
//         const char* name_; // Debug name
//
//         // Initialize the free list
//         void initializeFreeList();
//
// #ifdef _DEBUG
//         // Debug tracking for double-free detection
//         std::unordered_map<void*, bool> allocationMap_;
//         mutable std::mutex debugMutex_;
// #endif
//     };

    // ========================================================================
    // LINEAR ALLOCATOR (BUMP ALLOCATOR)
    // ========================================================================

    /**
     * @brief Linear allocator for sequential allocations
     *
     * Fastest possible allocation - just bumps a pointer.
     * Cannot free individual allocations, only reset entire allocator.
     * Perfect for level loading, initialization, or per-frame scratch memory.
     *
     * Performance characteristics:
     * - Allocation: O(1) - just increment pointer
     * - Deallocation: Not supported (only reset)
     * - No fragmentation
     * - Perfect cache locality for sequential access
     */
//     class LinearAllocator : public IAllocator {
//     public:
//         /**
//          * @brief Constructor with specified capacity
//          * @param capacity Total buffer size in bytes
//          * @param name Optional debug name
//          */
//         explicit LinearAllocator(MemorySize capacity, const char* name = "LinearAllocator");
//
//         /**
//          * @brief Destructor - frees backing memory
//          */
//         ~LinearAllocator() override;
//
//         // Delete copy operations
//         LinearAllocator(const LinearAllocator&) = delete;
//         LinearAllocator& operator=(const LinearAllocator&) = delete;
//
//         // Allow move operations
//         LinearAllocator(LinearAllocator&& other) noexcept;
//         LinearAllocator& operator=(LinearAllocator&& other) noexcept;
//
//         // IAllocator interface implementation
//         void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
//                        AllocationFlags flags = AllocationFlags::NONE) override;
//         void deallocate(void* ptr) override; // No-op for linear allocator
//         MemorySize getCapacity() const override { return capacity_; }
//         MemorySize getUsedMemory() const override { return current_.load(std::memory_order_acquire); }
//         void reset() override;
//         bool owns(const void* ptr) const override;
//         MemorySize getAllocationSize(const void* ptr) const override;
//         const char* getName() const override { return name_; }
//
//         /**
//          * @brief Get current allocation count
//          * @return Number of active allocations
//          */
//         std::size_t getAllocationCount() const {
//             return allocationCount_.load(std::memory_order_acquire);
//         }
//
//     private:
//         void* memory_; // Base memory pointer
//         MemorySize capacity_; // Total capacity
//         std::atomic<MemorySize> current_; // Current position
//         std::atomic<std::size_t> allocationCount_; // Number of allocations
//         const char* name_; // Debug name
//
// #ifdef _DEBUG
//         // Track allocations for debugging
//         struct DebugAllocation {
//             MemorySize offset;
//             MemorySize size;
//         };
//         std::vector<DebugAllocation> debugAllocations_;
//         mutable std::mutex debugMutex_;
// #endif
//     };

    // ========================================================================
    // RING BUFFER ALLOCATOR (CIRCULAR BUFFER)
    // ========================================================================

    /**
     * @brief Ring buffer allocator for streaming data
     *
     * Circular buffer that wraps around when reaching the end.
     * Perfect for temporary data with known lifetime, streaming assets,
     * or per-frame temporary allocations.
     *
     * Performance characteristics:
     * - Allocation: O(1) - advance write pointer
     * - Deallocation: O(1) - advance read pointer
     * - Automatic memory reuse
     * - Good for streaming patterns
     */
//     class RingBufferAllocator : public IAllocator {
//     public:
//         /**
//          * @brief Constructor with specified capacity
//          * @param capacity Total buffer size in bytes
//          * @param name Optional debug name
//          */
//         explicit RingBufferAllocator(MemorySize capacity, const char* name = "RingBufferAllocator");
//
//         /**
//          * @brief Destructor - frees backing memory
//          */
//         ~RingBufferAllocator() override;
//
//         // Delete copy operations
//         RingBufferAllocator(const RingBufferAllocator&) = delete;
//         RingBufferAllocator& operator=(const RingBufferAllocator&) = delete;
//
//         // Allow move operations
//         RingBufferAllocator(RingBufferAllocator&& other) noexcept;
//         RingBufferAllocator& operator=(RingBufferAllocator&& other) noexcept;
//
//         // IAllocator interface implementation
//         void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
//                        AllocationFlags flags = AllocationFlags::NONE) override;
//         void deallocate(void* ptr) override;
//         MemorySize getCapacity() const override { return capacity_; }
//         MemorySize getUsedMemory() const override;
//         void reset() override;
//         bool owns(const void* ptr) const override;
//         MemorySize getAllocationSize(const void* ptr) const override;
//         const char* getName() const override { return name_; }
//
//         /**
//          * @brief Mark current position for later release
//          * @return Fence value representing current position
//          */
//         std::uint64_t createFence();
//
//         /**
//          * @brief Wait for fence to be reached (free memory up to fence)
//          * @param fence Fence value to wait for
//          */
//         void waitForFence(std::uint64_t fence);
//
//         /**
//          * @brief Check if buffer can allocate without wrapping
//          * @param size Size to check
//          * @return True if allocation fits without wrap
//          */
//         bool canAllocateWithoutWrap(MemorySize size) const;
//
//     private:
//         // Allocation header for tracking
//         struct AllocationHeader {
//             MemorySize size;
//             std::uint64_t fence;
// #ifdef _DEBUG
//             std::uint32_t magic; // Magic number for validation
// #endif
//         };
//
//         void* memory_; // Base memory pointer
//         MemorySize capacity_; // Total capacity
//         std::atomic<MemorySize> head_; // Write position
//         std::atomic<MemorySize> tail_; // Read position
//         std::atomic<std::uint64_t> fenceCounter_; // Fence counter
//         const char* name_; // Debug name
//
//         // Thread safety for multi-producer
//         std::mutex allocationMutex_;
//
// #ifdef _DEBUG
//         static constexpr std::uint32_t MAGIC_NUMBER = 0xFEEDFACE;
// #endif
//     };

    // ========================================================================
    // BUDDY ALLOCATOR (BINARY BUDDY SYSTEM)
    // ========================================================================

    /**
     * @brief Buddy allocator for variable-sized allocations
     *
     * Splits memory into power-of-2 sized blocks using binary tree.
     * Good balance between fragmentation and allocation speed.
     * Useful for texture atlases, GPU memory management.
     *
     * Performance characteristics:
     * - Allocation: O(log n) - tree traversal
     * - Deallocation: O(log n) - coalescing
     * - Moderate fragmentation (internal)
     * - Power-of-2 size restriction
     */
    class BuddyAllocator : public IAllocator {
    public:
        /**
         * @brief Constructor with size constraints
         * @param minBlockSize Minimum allocatable block size (must be power of 2)
         * @param maxBlockSize Maximum block size / total capacity (must be power of 2)
         * @param name Optional debug name
         */
        BuddyAllocator(MemorySize minBlockSize,
                       MemorySize maxBlockSize,
                       const char* name = "BuddyAllocator");

        /**
         * @brief Destructor - frees backing memory
         */
        ~BuddyAllocator() override;

        // Delete copy operations
        BuddyAllocator(const BuddyAllocator&) = delete;
        BuddyAllocator& operator=(const BuddyAllocator&) = delete;

        // IAllocator interface implementation
        void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT,
                       AllocationFlags flags = AllocationFlags::NONE) override;
        void deallocate(void* ptr) override;
        MemorySize getCapacity() const override { return maxBlockSize_; }
        MemorySize getUsedMemory() const override;
        void reset() override;
        bool owns(const void* ptr) const override;
        MemorySize getAllocationSize(const void* ptr) const override;
        const char* getName() const override { return name_; }

        /**
         * @brief Get fragmentation ratio
         * @return Fragmentation as percentage (0-100)
         */
        float getFragmentation() const;

        /**
         * @brief Visualize memory layout for debugging
         * @return String representation of memory blocks
         */
        std::string visualize() const;

    private:
        // Block states in the buddy tree
        enum class BlockState : std::uint8_t {
            FREE, // Block is free
            SPLIT, // Block is split into buddies
            ALLOCATED // Block is allocated
        };

        // Node in the buddy tree
        struct BuddyNode {
            BlockState state;
            MemorySize size;
            void* address;
            BuddyNode* left; // Left child (buddy)
            BuddyNode* right; // Right child (buddy)
            BuddyNode* parent; // Parent node
        };

        void* memory_; // Base memory pointer
        MemorySize minBlockSize_; // Minimum block size
        MemorySize maxBlockSize_; // Maximum block size (capacity)
        BuddyNode* root_; // Root of buddy tree
        const char* name_; // Debug name

        // Free lists for each block size level
        std::vector<std::vector<BuddyNode*>> freeLists_;
        mutable std::shared_mutex treeMutex_; // Protect tree operations

        // Helper methods
        std::size_t getSizeLevel(MemorySize size) const;
    };

    // ========================================================================
    // MEMORY MANAGER CONFIGURATION
    // ========================================================================

    /**
     * @brief Configuration for memory manager initialization
     */
    // struct MemoryManagerConfig {
    //     // Main heap configuration
    //     MemorySize mainHeapSize = 512 * 1024 * 1024; // 512 MB default
    //     MemorySize debugHeapSize = 64 * 1024 * 1024; // 64 MB for debug
    //
    //     // Per-frame allocators
    //     MemorySize frameStackSize = 16 * 1024 * 1024; // 16 MB per frame
    //     MemorySize frameLinearSize = 32 * 1024 * 1024; // 32 MB linear per frame
    //     std::uint8_t frameBufferCount = 3; // Triple buffering
    //
    //     // Specialized allocators
    //     MemorySize renderingPoolSize = 128 * 1024 * 1024; // 128 MB for rendering
    //     MemorySize physicsPoolSize = 64 * 1024 * 1024; // 64 MB for physics
    //     MemorySize audioRingBufferSize = 32 * 1024 * 1024; // 32 MB for audio streaming
    //     MemorySize networkBufferSize = 16 * 1024 * 1024; // 16 MB for networking
    //
    //     // Pool configurations
    //     struct PoolConfig {
    //         MemorySize blockSize;
    //         std::size_t blockCount;
    //         MemoryCategory category;
    //     };
    //
    //     std::vector<PoolConfig> customPools;
    //
    //     // Profiling and debugging
    //     bool enableProfiling = true; // Track allocations
    //     bool enableMemoryTagging = true; // Tag allocations with categories
    //     bool enableLeakDetection = true; // Detect memory leaks
    //     bool enableBoundsChecking = true; // Check for buffer overruns
    //     bool fillFreedMemory = true; // Fill freed memory with pattern
    //     std::uint8_t freedMemoryPattern = 0xDE; // Pattern for freed memory
    //     std::uint8_t uninitializedPattern = 0xCD; // Pattern for new allocations
    //
    //     // Memory limits and policies
    //     MemorySize lowMemoryThreshold = 100 * 1024 * 1024; // 100 MB threshold
    //     MemorySize criticalMemoryThreshold = 50 * 1024 * 1024; // 50 MB critical
    //     bool allowSystemFallback = true; // Fall back to system malloc
    //     bool preallocateMemory = true; // Preallocate all memory upfront
    // };

    // ========================================================================
    // SCOPED ALLOCATOR (RAII WRAPPER)
    // ========================================================================

    /**
     * @brief RAII wrapper for automatic allocator restoration
     *
     * Automatically restores allocator state when going out of scope.
     * Perfect for temporary allocations within a function or scope.
     */
    // class ScopedAllocator {
    // public:
    //     /**
    //      * @brief Constructor saves current state
    //      * @param allocator Allocator to use in this scope
    //      */
    //     explicit ScopedAllocator(StackAllocator& allocator)
    //         : allocator_(allocator)
    //           , marker_(allocator.getMarker()) {
    //     }
    //
    //     /**
    //      * @brief Destructor restores allocator state
    //      */
    //     ~ScopedAllocator() {
    //         allocator_.freeToMarker(marker_);
    //     }
    //
    //     // Delete copy/move operations - RAII object
    //     ScopedAllocator(const ScopedAllocator&) = delete;
    //     ScopedAllocator& operator=(const ScopedAllocator&) = delete;
    //     ScopedAllocator(ScopedAllocator&&) = delete;
    //     ScopedAllocator& operator=(ScopedAllocator&&) = delete;
    //
    //     /**
    //      * @brief Allocate memory within this scope
    //      * @param size Bytes to allocate
    //      * @param alignment Alignment requirement
    //      * @return Allocated memory pointer
    //      */
    //     void* allocate(MemorySize size, MemorySize alignment = DEFAULT_ALIGNMENT) {
    //         return allocator_.allocate(size, alignment);
    //     }
    //
    // private:
    //     StackAllocator& allocator_;
    //     StackAllocator::Marker marker_;
    // };


    // ========================================================================
    // MEMORY MANAGER (MAIN SYSTEM)
    // ========================================================================

    /**
     * @brief Central memory management system for the game engine
     *
     * Manages all memory allocators, provides allocation strategies,
     * tracks memory usage, and handles memory pressure situations.
     *
     * Features:
     * - Multiple allocator types for different use cases
     * - Per-frame allocators with automatic reset
     * - Memory profiling and leak detection
     * - Thread-safe allocation
     * - Memory pressure callbacks
     * - Allocation replay for debugging
     */
//     class MemoryManager {
//     public:
//         // Singleton pattern (or can be created as needed)
//         static MemoryManager& getInstance() {
//             static MemoryManager instance;
//             return instance;
//         }
//
//         /**
//          * @brief Initialize memory manager with configuration
//          * @param config Configuration parameters
//          * @return True if initialization successful
//          */
//         bool initialize(const MemoryManagerConfig& config);
//
//         /**
//          * @brief Shutdown memory manager and free all memory
//          */
//         void shutdown();
//
//         /**
//          * @brief Check if memory manager is initialized
//          * @return True if initialized
//          */
//         bool isInitialized() const { return initialized_; }
//
//         // ====================================================================
//         // ALLOCATION INTERFACE
//         // ====================================================================
//
//         /**
//          * @brief Allocate memory from appropriate allocator
//          * @param size Bytes to allocate
//          * @param category Memory category for tracking
//          * @param alignment Alignment requirement
//          * @param flags Allocation flags
//          * @return Allocated memory pointer, or nullptr on failure
//          */
//         void* allocate(MemorySize size,
//                        MemoryCategory category = MemoryCategory::GENERAL,
//                        MemorySize alignment = DEFAULT_ALIGNMENT,
//                        AllocationFlags flags = AllocationFlags::NONE);
//
//         /**
//          * @brief Deallocate previously allocated memory
//          * @param ptr Memory to free
//          * @param category Category used for allocation
//          */
//         void deallocate(void* ptr, MemoryCategory category = MemoryCategory::GENERAL);
//
//         /**
//          * @brief Reallocate memory to new size
//          * @param ptr Original pointer
//          * @param newSize New size in bytes
//          * @param category Memory category
//          * @param alignment Alignment requirement
//          * @return Reallocated pointer, or nullptr on failure
//          */
//         void* reallocate(void* ptr, MemorySize newSize,
//                          MemoryCategory category = MemoryCategory::GENERAL,
//                          MemorySize alignment = DEFAULT_ALIGNMENT);
//
//         // ====================================================================
//         // TYPED ALLOCATION HELPERS
//         // ====================================================================
//
//         /**
//          * @brief Allocate and construct single object
//          * @tparam T Object type
//          * @tparam Args Constructor argument types
//          * @param category Memory category
//          * @param args Constructor arguments
//          * @return Pointer to constructed object
//          */
//         template <typename T, typename... Args>
//         T* allocateObject(MemoryCategory category, Args&&... args) {
//             void* memory = allocate(sizeof(T), category, alignof(T));
//             if (!memory) return nullptr;
//
//             try {
//                 return new(memory) T(std::forward<Args>(args)...);
//             }
//             catch (...) {
//                 deallocate(memory, category);
//                 throw;
//             }
//         }
//
//         /**
//          * @brief Deallocate and destruct single object
//          * @tparam T Object type
//          * @param ptr Object to destroy
//          * @param category Memory category used for allocation
//          */
//         template <typename T>
//         void deallocateObject(T* ptr, MemoryCategory category) {
//             if (!ptr) return;
//
//             ptr->~T();
//             deallocate(ptr, category);
//         }
//
//         /**
//          * @brief Allocate array of objects
//          * @tparam T Element type
//          * @param count Number of elements
//          * @param category Memory category
//          * @return Pointer to array
//          */
//         template <typename T>
//         T* allocateArray(std::size_t count, MemoryCategory category) {
//             MemorySize size = sizeof(T) * count + sizeof(std::size_t);
//             void* memory = allocate(size, category, alignof(T));
//             if (!memory) return nullptr;
//
//             // Store count at beginning
//             *reinterpret_cast<std::size_t*>(memory) = count;
//             T* array = reinterpret_cast<T*>(reinterpret_cast<std::uint8_t*>(memory) + sizeof(std::size_t));
//
//             // Default construct elements
//             for (std::size_t i = 0; i < count; ++i) {
//                 new(&array[i]) T();
//             }
//
//             return array;
//         }
//
//         /**
//          * @brief Deallocate array of objects
//          * @tparam T Element type
//          * @param array Array to destroy
//          * @param category Memory category used for allocation
//          */
//         template <typename T>
//         void deallocateArray(T* array, MemoryCategory category) {
//             if (!array) return;
//
//             // Get count from before array
//             void* memory = reinterpret_cast<std::uint8_t*>(array) - sizeof(std::size_t);
//             std::size_t count = *reinterpret_cast<std::size_t*>(memory);
//
//             // Destruct elements
//             for (std::size_t i = 0; i < count; ++i) {
//                 array[i].~T();
//             }
//
//             deallocate(memory, category);
//         }
//
//         // ====================================================================
//         // FRAME ALLOCATORS
//         // ====================================================================
//
//         /**
//          * @brief Get current frame's stack allocator
//          * @return Reference to frame stack allocator
//          */
//         StackAllocator& getFrameStackAllocator();
//
//         /**
//          * @brief Get current frame's linear allocator
//          * @return Reference to frame linear allocator
//          */
//         LinearAllocator& getFrameLinearAllocator();
//
//         /**
//          * @brief Begin new frame (swap frame allocators)
//          * @param frameNumber Current frame number
//          */
//         void beginFrame(std::uint64_t frameNumber);
//
//         /**
//          * @brief End current frame (reset old frame allocators)
//          */
//         void endFrame();
//
//         // ====================================================================
//         // SPECIALIZED ALLOCATORS
//         // ====================================================================
//
//         /**
//          * @brief Get allocator for specific category
//          * @param category Memory category
//          * @return Pointer to allocator, or nullptr if not available
//          */
//         IAllocator* getAllocator(MemoryCategory category);
//
//         /**
//          * @brief Register custom allocator for category
//          * @param category Memory category
//          * @param allocator Allocator to register
//          */
//         void registerAllocator(MemoryCategory category, std::unique_ptr<IAllocator> allocator);
//
//         /**
//          * @brief Create pool allocator for specific object type
//          * @tparam T Object type
//          * @param objectCount Number of objects in pool
//          * @param category Memory category
//          * @return Pointer to created pool allocator
//          */
//         template <typename T>
//         PoolAllocator* createPoolAllocator(std::size_t objectCount,
//                                            MemoryCategory category = MemoryCategory::GENERAL) {
//             auto pool = std::make_unique<PoolAllocator>(
//                 sizeof(T), objectCount, alignof(T),
//                 (std::string("Pool_") + typeid(T).name()).c_str()
//             );
//
//             PoolAllocator* poolPtr = pool.get();
//             registerAllocator(category, std::move(pool));
//             return poolPtr;
//         }
//
//         // ====================================================================
//         // MEMORY STATISTICS AND PROFILING
//         // ====================================================================
//
//         /**
//          * @brief Get total memory usage across all allocators
//          * @return Total bytes allocated
//          */
//         MemorySize getTotalMemoryUsage() const;
//
//         /**
//          * @brief Get memory usage for specific category
//          * @param category Memory category
//          * @return Bytes allocated in category
//          */
//         MemorySize getCategoryMemoryUsage(MemoryCategory category) const;
//
//         /**
//          * @brief Get global memory statistics
//          * @return Memory statistics structure
//          */
//         const MemoryStats& getGlobalStats() const { return globalStats_; }
//
//         /**
//          * @brief Generate memory report
//          * @return String containing detailed memory report
//          */
//         std::string generateMemoryReport() const;
//
//         /**
//          * @brief Dump memory allocations to a file
//          * @param filename Output filename
//          * @return True if successful
//          */
//         bool dumpAllocations(const std::string& filename) const;
//
//         /**
//          * @brief Check for memory leaks
//          * @return Number of leaked allocations
//          */
//         std::size_t checkForLeaks() const;
//
//         // ====================================================================
//         // MEMORY PRESSURE HANDLING
//         // ====================================================================
//
//         /**
//          * @brief Memory pressure callback function type
//          */
//         using MemoryPressureCallback = std::function<void(MemorySize available, MemorySize required)>;
//
//         /**
//          * @brief Register callback for low memory situations
//          * @param callback Function to call on low memory
//          */
//         void registerMemoryPressureCallback(MemoryPressureCallback callback);
//
//         /**
//          * @brief Trigger memory cleanup
//          * @param targetBytes Bytes to try to free
//          * @return Actual bytes freed
//          */
//         MemorySize performMemoryCleanup(MemorySize targetBytes);
//
//         /**
//          * @brief Get system memory information
//          * @param totalPhysical Output total physical memory
//          * @param availablePhysical Output available physical memory
//          * @param totalVirtual Output total virtual memory
//          * @param availableVirtual Output available virtual memory
//          */
//         static void getSystemMemoryInfo(MemorySize& totalPhysical,
//                                         MemorySize& availablePhysical,
//                                         MemorySize& totalVirtual,
//                                         MemorySize& availableVirtual);
//
//     private:
//         // Private constructor for singleton
//         MemoryManager() = default;
//         ~MemoryManager();
//
//         // Delete copy/move operations
//         MemoryManager(const MemoryManager&) = delete;
//         MemoryManager& operator=(const MemoryManager&) = delete;
//
//         // Configuration
//         MemoryManagerConfig config_;
//         bool initialized_ = false;
//
//         // Main allocators
//         std::unique_ptr<IAllocator> mainHeap_;
//         std::unique_ptr<IAllocator> debugHeap_;
//
//         // Frame allocators (multi-buffered)
//         struct FrameAllocators {
//             std::unique_ptr<StackAllocator> stackAllocator;
//             std::unique_ptr<LinearAllocator> linearAllocator;
//             std::uint64_t frameNumber = 0;
//         };
//
//         std::vector<FrameAllocators> frameAllocators_;
//         std::atomic<std::uint8_t> currentFrameIndex_{0};
//
//         // Category-specific allocators
//         std::array<std::unique_ptr<IAllocator>,
//                    static_cast<std::size_t>(MemoryCategory::COUNT)> categoryAllocators_;
//
//         // Custom allocators
//         std::unordered_map<std::string, std::unique_ptr<IAllocator>> customAllocators_;
//
//         // Global statistics
//         mutable MemoryStats globalStats_;
//
//         // Allocation tracking (debug mode)
// #ifdef _DEBUG
//         struct AllocationRecord {
//             void* address;
//             MemorySize size;
//             MemoryCategory category;
//             std::string file;
//             int line;
//             std::chrono::steady_clock::time_point timestamp;
//             std::thread::id threadId;
//         };
//         std::unordered_map<void*, AllocationRecord> allocationRecords_;
//         mutable std::shared_mutex recordsMutex_;
// #endif
//
//         // Memory pressure handling
//         std::vector<MemoryPressureCallback> memoryPressureCallbacks_;
//         std::atomic<bool> inMemoryPressure_{false};
//
//         // Thread safety
//         mutable std::shared_mutex allocatorsMutex_;
//
//         // Helper methods
//         IAllocator* selectAllocator(MemoryCategory category, MemorySize size, AllocationFlags flags);
//         void updateGlobalStats(const MemoryStats& allocatorStats);
//         void handleAllocationFailure(MemorySize size, MemoryCategory category);
//
// #ifdef _DEBUG
//         void recordAllocation(void* ptr, MemorySize size, MemoryCategory category,
//                               const char* file, int line);
//         void removeAllocationRecord(void* ptr);
//         void validateAllocation(void* ptr) const;
// #endif
//     };

    // ========================================================================
    // MEMORY ALLOCATION MACROS
    // ========================================================================

    // Debug mode allocation tracking
// #ifdef _DEBUG
// #define ENGINE_ALLOC(size, category) \
// engine::memory::MemoryManager::getInstance().allocate(size, category, \
// engine::memory::DEFAULT_ALIGNMENT, engine::memory::AllocationFlags::NONE)
//
// #define ENGINE_FREE(ptr, category) \
// engine::memory::MemoryManager::getInstance().deallocate(ptr, category)
//
// #define ENGINE_NEW(type, category, ...) \
// engine::memory::MemoryManager::getInstance().allocateObject<type>(category, ##__VA_ARGS__)
//
// #define ENGINE_DELETE(ptr, type, category) \
// engine::memory::MemoryManager::getInstance().deallocateObject<type>(ptr, category)
// #else
// #define ENGINE_ALLOC(size, category) \
// engine::memory::MemoryManager::getInstance().allocate(size, category)
//
// #define ENGINE_FREE(ptr, category) \
// engine::memory::MemoryManager::getInstance().deallocate(ptr, category)
//
// #define ENGINE_NEW(type, category, ...) \
// engine::memory::MemoryManager::getInstance().allocateObject<type>(category, ##__VA_ARGS__)
//
// #define ENGINE_DELETE(ptr, type, category) \
// engine::memory::MemoryManager::getInstance().deallocateObject<type>(ptr, category)
// #endif
//
//     // ========================================================================
//     // STL ALLOCATOR ADAPTER
//     // ========================================================================
//
//     /**
//      * @brief STL-compatible allocator that uses our memory system
//      *
//      * Allows STL containers to use our custom memory management.
//      * Example: std::vector<int, StlAllocator<int>> myVector;
//      */
//     template <typename T>
//     class StlAllocator {
//     public:
//         using value_type = T;
//         using pointer = T*;
//         using const_pointer = const T*;
//         using reference = T&;
//         using const_reference = const T&;
//         using size_type = std::size_t;
//         using difference_type = std::ptrdiff_t;
//
//         // Rebind support
//         template <typename U>
//         struct rebind {
//             using other = StlAllocator<U>;
//         };
//
//         // Constructors
//         StlAllocator(MemoryCategory category = MemoryCategory::GENERAL) noexcept
//             : category_(category) {
//         }
//
//         template <typename U>
//         StlAllocator(const StlAllocator<U>& other) noexcept
//             : category_(other.category_) {
//         }
//
//         // Allocation functions
//         T* allocate(size_type n) {
//             void* memory = MemoryManager::getInstance().allocate(
//                 n * sizeof(T), category_, alignof(T)
//             );
//             if (!memory) {
//                 throw std::bad_alloc();
//             }
//             return static_cast<T*>(memory);
//         }
//
//         void deallocate(T* ptr, size_type n) noexcept {
//             MemoryManager::getInstance().deallocate(ptr, category_);
//         }
//
//         // Comparison operators
//         template <typename U>
//         bool operator==(const StlAllocator<U>& other) const noexcept {
//             return category_ == other.category_;
//         }
//
//         template <typename U>
//         bool operator!=(const StlAllocator<U>& other) const noexcept {
//             return !(*this == other);
//         }
//
//     private:
//         MemoryCategory category_;
//
//         template <typename U>
//         friend class StlAllocator;
//     };
// } // namespace engine::memory
