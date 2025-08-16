//
// Created by Andres Guerrero on 11-08-25.
//
//
// #include "./MemoryManager.h"
// #include "manager/MemoryManager.h"
//
// #include <cstring>          // For memcpy, memset
// #include <iostream>         // For error logging
// #include <sstream>          // For string building
// #include <iomanip>          // For formatting
// #include <fstream>          // For file output
// #include <thread>           // For thread ID
//
// #include "allocators/LinearAllocator.h"
// #include "allocators/RingBufferAllocator.h"
//
// #ifdef _WIN32
// #include <windows.h>
// #include <psapi.h>      // For memory info
// #elif __APPLE__
// #include <mach/mach.h>
// #include <sys/sysctl.h>
// #elif __linux__
// #include <sys/sysinfo.h>
// #include <unistd.h>
// #endif

// namespace engine::memory {
    // ========================================================================
    // STACK ALLOCATOR IMPLEMENTATION
    // ========================================================================
//
//     StackAllocator::StackAllocator(MemorySize capacity, const char* name)
//         : memory(nullptr)
//           , capacity(capacity)
//           , current(0)
//           , highWaterMark(0)
//           , name(name) {
//         // Allocate backing memory with proper alignment
//         // Using aligned_alloc for C++17 compliance
//         memory = std::aligned_alloc(CACHE_LINE_SIZE, capacity);
//
//         if (!memory) {
//             throw std::bad_alloc();
//         }
//
//         // Initialize memory with debug pattern in debug builds
// #ifdef _DEBUG
//         std::memset(memory, 0xCD, capacity); // Uninitialized memory pattern
// #endif
//
//         // Initialize statistics
//         stats.currentUsage = 0;
//         stats.peakUsage = 0;
//         stats.totalAllocated = 0;
//         stats.allocationCount = 0;
//     }
//
//     StackAllocator::~StackAllocator() {
//         if (memory) {
//             // Check for leaks in debug mode
// #ifdef _DEBUG
//             if (current.load() > 0) {
//                 std::cerr << "Warning: StackAllocator '" << name
//                     << "' destroyed with " << current.load()
//                     << " bytes still allocated!" << std::endl;
//             }
// #endif
//
//             std::free(memory);
//             memory = nullptr;
//         }
//     }
//
//     StackAllocator::StackAllocator(StackAllocator&& other) noexcept
//         : memory(other.memory)
//           , capacity(other.capacity)
//           , current(other.current.load())
//           , highWaterMark(other.highWaterMark.load())
//           , name(other.name) {
//         other.memory = nullptr;
//         other.capacity = 0;
//         other.current = 0;
//         other.highWaterMark = 0;
//     }
//
//     StackAllocator& StackAllocator::operator=(StackAllocator&& other) noexcept {
//         if (this != &other) {
//             // Free existing memory
//             if (memory) {
//                 std::free(memory);
//             }
//
//             // Move from other
//             memory = other.memory;
//             capacity = other.capacity;
//             current = other.current.load();
//             highWaterMark = other.highWaterMark.load();
//             name = other.name;
//
//             // Reset other
//             other.memory = nullptr;
//             other.capacity = 0;
//             other.current = 0;
//             other.highWaterMark = 0;
//         }
//         return *this;
//     }
//
//     void* StackAllocator::allocate(MemorySize size, MemorySize alignment, AllocationFlags flags) {
//         // Validate alignment is power of 2
//         assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");
//
//         // Calculate total size including header
//         constexpr MemorySize headerSize = sizeof(AllocationHeader);
//         const MemorySize totalSize = size + headerSize;
//
//         // Get current position and calculate aligned address
//         MemorySize currentPos = current.load(std::memory_order_acquire);
//         void* currentAddress = static_cast<std::uint8_t*>(memory) + currentPos;
//         void* alignedAddress = alignPointer(currentAddress, alignment);
//
//         // Calculate adjustment for alignment
//         const MemorySize adjustment = reinterpret_cast<std::uint8_t*>(alignedAddress) -
//             reinterpret_cast<std::uint8_t*>(currentAddress);
//
//         // Calculate new position after allocation
//         MemorySize newPos = currentPos + adjustment + totalSize;
//
//         // Check if allocation fits
//         if (newPos > capacity) {
//             stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//             return nullptr;
//         }
//
//         // Atomic update of current position
//         MemorySize expected = currentPos;
//         while (!current.compare_exchange_weak(expected, newPos,
//                                                std::memory_order_acq_rel,
//                                                std::memory_order_acquire)) {
//             // Another thread allocated, recalculate
//             currentPos = expected;
//             currentAddress = reinterpret_cast<std::uint8_t*>(memory) + currentPos;
//             alignedAddress = alignPointer(currentAddress, alignment);
//             const MemorySize newAdjustment = reinterpret_cast<std::uint8_t*>(alignedAddress) -
//                 reinterpret_cast<std::uint8_t*>(currentAddress);
//             const MemorySize newNewPos = currentPos + newAdjustment + totalSize;
//
//             if (newNewPos > capacity) {
//                 stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//                 return nullptr;
//             }
//
//             newPos = newNewPos;
//         }
//
//         // Update high water mark
//         MemorySize currentHigh = highWaterMark.load(std::memory_order_acquire);
//         while (newPos > currentHigh &&
//             !highWaterMark.compare_exchange_weak(currentHigh, newPos,
//                                                   std::memory_order_acq_rel,
//                                                   std::memory_order_acquire)) {
//             // Keep trying
//         }
//
//         // Write allocation header
//         AllocationHeader* header = reinterpret_cast<AllocationHeader*>(alignedAddress);
//         header->size = size;
//         header->adjustment = adjustment;
// #ifdef _DEBUG
//         header->sentinel = SENTINEL_VALUE;
//         allocationCount_.fetch_add(1, std::memory_order_relaxed);
// #endif
//
//         // Get user pointer (after header)
//         void* userPtr = reinterpret_cast<std::uint8_t*>(alignedAddress) + headerSize;
//
//         // Handle allocation flags
//         if (hasFlag(flags, AllocationFlags::ZERO_MEMORY)) {
//             std::memset(userPtr, 0, size);
//         }
// #ifdef _DEBUG
//         else if (hasFlag(flags, AllocationFlags::DEBUG_FILL)) {
//             std::memset(userPtr, 0xAB, size); // Allocated memory pattern
//         }
// #endif
//
//         // Update statistics
//         recordAllocation(size + adjustment + headerSize);
//
//         return userPtr;
//     }
//
//     void StackAllocator::deallocate(void* ptr) {
//         if (!ptr) return;
//
//         // Stack allocator only supports freeing from the top
//         // Check if this is the most recent allocation
// #ifdef _DEBUG
//         // In debug mode, verify the allocation
//         AllocationHeader* header = reinterpret_cast<AllocationHeader*>(
//             reinterpret_cast<std::uint8_t*>(ptr) - sizeof(AllocationHeader)
//         );
//
//         if (header->sentinel != SENTINEL_VALUE) {
//             std::cerr << "Error: Corrupted allocation header in StackAllocator!" << std::endl;
//             assert(false && "Memory corruption detected");
//         }
//
//         // Calculate if this is the top allocation
//         void* headerAddress = header;
//         MemorySize allocStart = reinterpret_cast<std::uint8_t*>(headerAddress) -
//             reinterpret_cast<std::uint8_t*>(memory);
//         MemorySize allocEnd = allocStart + header->adjustment + sizeof(AllocationHeader) + header->size;
//
//         if (allocEnd == current.load(std::memory_order_acquire)) {
//             // This is the top allocation, we can free it
//             current.store(allocStart, std::memory_order_release);
//             recordDeallocation(header->size + header->adjustment + sizeof(AllocationHeader));
//             allocationCount_.fetch_sub(1, std::memory_order_relaxed);
//
//             // Fill with freed pattern
//             std::memset(headerAddress, 0xDE, allocEnd - allocStart);
//         }
//         else {
//             // Not the top allocation - can't free individual allocations in stack
//             std::cerr << "Warning: Attempted to free non-top allocation in StackAllocator. "
//                 << "Use freeToMarker() or reset() instead." << std::endl;
//         }
// #else
//         // In release mode, stack doesn't support individual deallocation
//         // This is a no-op to maintain the stack property
// #endif
//     }
//
//     void StackAllocator::reset() {
// #ifdef _DEBUG
//         // Fill freed memory with pattern
//         MemorySize used = current.load(std::memory_order_acquire);
//         if (used > 0) {
//             std::memset(memory, 0xCD, used); // Reset to uninitialized pattern
//         }
//         allocationCount_.store(0, std::memory_order_release);
// #endif
//
//         // Reset to beginning
//         current.store(0, std::memory_order_release);
//
//         // Update statistics
//         stats.currentUsage.store(0, std::memory_order_release);
//     }
//
//     void StackAllocator::freeToMarker(Marker marker) {
//         MemorySize currentPos = current.load(std::memory_order_acquire);
//
//         if (marker > currentPos) {
//             std::cerr << "Error: Invalid marker in StackAllocator::freeToMarker!" << std::endl;
//             return;
//         }
//
// #ifdef _DEBUG
//         // Fill freed memory with pattern
//         MemorySize freedSize = currentPos - marker;
//         if (freedSize > 0) {
//             void* freedStart = reinterpret_cast<std::uint8_t*>(memory) + marker;
//             std::memset(freedStart, 0xDE, freedSize);
//         }
// #endif
//
//         // Update position
//         current.store(marker, std::memory_order_release);
//
//         // Update statistics
//         recordDeallocation(currentPos - marker);
//     }
//
//     bool StackAllocator::owns(const void* ptr) const {
//         if (!ptr || !memory) return false;
//
//         const std::uint8_t* bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);
//         const std::uint8_t* memStart = reinterpret_cast<const std::uint8_t*>(memory);
//         const std::uint8_t* memEnd = memStart + capacity;
//
//         return bytePtr >= memStart && bytePtr < memEnd;
//     }

//     MemorySize StackAllocator::getAllocationSize(const void* ptr) const {
//         if (!owns(ptr)) return 0;
//
// #ifdef _DEBUG
//         // In debug mode, we can get the actual size from the header
//         const AllocationHeader* header = reinterpret_cast<const AllocationHeader*>(
//             reinterpret_cast<const std::uint8_t*>(ptr) - sizeof(AllocationHeader)
//         );
//
//         if (header->sentinel == SENTINEL_VALUE) {
//             return header->size;
//         }
// #endif
//
//         // Without debug info, we can't determine individual allocation sizes
//         return 0;
//     }

    // ========================================================================
    // POOL ALLOCATOR IMPLEMENTATION
    // ========================================================================

//     PoolAllocator::PoolAllocator(MemorySize blockSize,
//                                  std::size_t blockCount,
//                                  MemorySize alignment,
//                                  const char* name)
//         : memory_(nullptr)
//           , blockSize_(alignSize(blockSize, alignment)) // Ensure block size is aligned
//           , blockCount_(blockCount)
//           , alignment_(alignment)
//           , freeList_(nullptr)
//           , usedBlocks_(0)
//           , name_(name) {
//         // Validate parameters
//         assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");
//         assert(blockSize >= sizeof(FreeNode) && "Block size must be at least pointer size");
//         assert(blockCount > 0 && "Block count must be greater than 0");
//
//         // Calculate total memory needed
//         const MemorySize totalSize = blockSize_ * blockCount_;
//
//         // Allocate aligned memory
//         memory_ = std::aligned_alloc(alignment, totalSize);
//
//         if (!memory_) {
//             throw std::bad_alloc();
//         }
//
//         // Initialize memory with debug pattern
// #ifdef _DEBUG
//         std::memset(memory_, 0xCD, totalSize);
// #endif
//
//         // Initialize free list
//         initializeFreeList();
//
//         // Initialize statistics
//         stats.currentUsage = 0;
//         stats.peakUsage = 0;
//         stats.totalAllocated = 0;
//         stats.allocationCount = 0;
//     }
//
//     PoolAllocator::~PoolAllocator() {
//         if (memory_) {
// #ifdef _DEBUG
//             // Check for memory leaks
//             if (usedBlocks_.load() > 0) {
//                 std::cerr << "Warning: PoolAllocator '" << name_
//                     << "' destroyed with " << usedBlocks_.load()
//                     << " blocks still allocated!" << std::endl;
//
//                 // In debug mode, verify all allocations were freed
//                 std::lock_guard<std::mutex> lock(debugMutex_);
//                 for (const auto& [ptr, allocated] : allocationMap_) {
//                     if (allocated) {
//                         std::cerr << "  Leaked block at: " << ptr << std::endl;
//                     }
//                 }
//             }
// #endif
//
//             std::free(memory_);
//             memory_ = nullptr;
//         }
//     }
//
//     PoolAllocator::PoolAllocator(PoolAllocator&& other) noexcept
//         : memory_(other.memory_)
//           , blockSize_(other.blockSize_)
//           , blockCount_(other.blockCount_)
//           , alignment_(other.alignment_)
//           , freeList_(other.freeList_.load())
//           , usedBlocks_(other.usedBlocks_.load())
//           , name_(other.name_) {
//         other.memory_ = nullptr;
//         other.blockSize_ = 0;
//         other.blockCount_ = 0;
//         other.freeList_ = nullptr;
//         other.usedBlocks_ = 0;
//
// #ifdef _DEBUG
//         // Move debug allocation map
//         std::lock_guard<std::mutex> lock(other.debugMutex_);
//         allocationMap_ = std::move(other.allocationMap_);
// #endif
//     }
//
//     PoolAllocator& PoolAllocator::operator=(PoolAllocator&& other) noexcept {
//         if (this != &other) {
//             // Free existing memory
//             if (memory_) {
//                 std::free(memory_);
//             }
//
//             // Move from other
//             memory_ = other.memory_;
//             blockSize_ = other.blockSize_;
//             blockCount_ = other.blockCount_;
//             alignment_ = other.alignment_;
//             freeList_ = other.freeList_.load();
//             usedBlocks_ = other.usedBlocks_.load();
//             name_ = other.name_;
//
//             // Reset other
//             other.memory_ = nullptr;
//             other.blockSize_ = 0;
//             other.blockCount_ = 0;
//             other.freeList_ = nullptr;
//             other.usedBlocks_ = 0;
//
// #ifdef _DEBUG
//             // Move debug allocation map
//             std::lock_guard<std::mutex> lock(other.debugMutex_);
//             allocationMap_ = std::move(other.allocationMap_);
// #endif
//         }
//         return *this;
//     }
//
//     void PoolAllocator::initializeFreeList() {
//         // Initialize all blocks as free
//         auto* currentBlock = reinterpret_cast<std::uint8_t*>(memory_);
//
//         // Build linked list of free blocks
//         FreeNode* firstNode = reinterpret_cast<FreeNode*>(currentBlock);
//         FreeNode* currentNode = firstNode;
//
//         for (std::size_t i = 0; i < blockCount_ - 1; ++i) {
//             FreeNode* nextNode = reinterpret_cast<FreeNode*>(currentBlock + blockSize_);
//             currentNode->next = nextNode;
//             currentNode = nextNode;
//             currentBlock += blockSize_;
//         }
//
//         // Last block points to null
//         currentNode->next = nullptr;
//
//         // Set head of free list
//         freeList_.store(firstNode, std::memory_order_release);
//
// #ifdef _DEBUG
//         // Initialize allocation tracking
//         std::lock_guard<std::mutex> lock(debugMutex_);
//         allocationMap_.clear();
//         std::uint8_t* block = reinterpret_cast<std::uint8_t*>(memory_);
//         for (std::size_t i = 0; i < blockCount_; ++i) {
//             allocationMap_[block] = false; // All blocks start as free
//             block += blockSize_;
//         }
// #endif
//     }
//
//     void* PoolAllocator::allocate(const MemorySize size, const MemorySize alignment, const AllocationFlags flags) {
//         // Check if requested size fits in a block
//         if (size > blockSize_) {
//             stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//             return nullptr;
//         }
//
//         // Check alignment compatibility
//         if (alignment > alignment_) {
//             stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//             return nullptr;
//         }
//
//         // Lock-free allocation using compare-and-swap
//         FreeNode* head = freeList_.load(std::memory_order_acquire);
//
//         while (head != nullptr) {
//             FreeNode* next = head->next;
//
//             // Try to update free list head
//             if (freeList_.compare_exchange_weak(head, next,
//                                                 std::memory_order_acq_rel,
//                                                 std::memory_order_acquire)) {
//                 // Successfully allocated block
//                 void* block = reinterpret_cast<void*>(head);
//
//                 // Update statistics
//                 usedBlocks_.fetch_add(1, std::memory_order_relaxed);
//                 recordAllocation(blockSize_);
//
// #ifdef _DEBUG
//                 // Track allocation
//                 {
//                     std::lock_guard<std::mutex> lock(debugMutex_);
//                     allocationMap_[block] = true;
//                 }
// #endif
//
//                 // Handle allocation flags
//                 if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
//                     std::memset(block, 0, blockSize_);
//                 }
// #ifdef _DEBUG
//                 else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
//                     std::memset(block, 0xAB, blockSize_); // Allocated pattern
//                 }
// #endif
//
//                 return block;
//             }
//
//             // CAS failed, retry with new head
//             head = freeList_.load(std::memory_order_acquire);
//         }
//
//         // No free blocks available
//         stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//         return nullptr;
//     }
//
//     void PoolAllocator::deallocate(void* ptr) {
//         if (!ptr) return;
//
//         // Verify pointer is from this pool
//         if (!owns(ptr)) {
//             std::cerr << "Error: Attempted to deallocate pointer not owned by this pool!" << std::endl;
//             assert(false && "Invalid deallocation");
//             return;
//         }
//
// #ifdef _DEBUG
//         // Check for double-free
//         {
//             std::lock_guard<std::mutex> lock(debugMutex_);
//             auto it = allocationMap_.find(ptr);
//             if (it == allocationMap_.end() || !it->second) {
//                 std::cerr << "Error: Double-free detected in PoolAllocator!" << std::endl;
//                 assert(false && "Double-free detected");
//                 return;
//             }
//             it->second = false; // Mark as free
//         }
//
//         // Fill with freed pattern
//         std::memset(ptr, 0xDE, blockSize_);
// #endif
//
//         // Add block back to free list (lock-free)
//         FreeNode* node = reinterpret_cast<FreeNode*>(ptr);
//         FreeNode* head = freeList_.load(std::memory_order_acquire);
//
//         do {
//             node->next = head;
//         }
//         while (!freeList_.compare_exchange_weak(head, node,
//                                                 std::memory_order_acq_rel,
//                                                 std::memory_order_acquire));
//
//         // Update statistics
//         usedBlocks_.fetch_sub(1, std::memory_order_relaxed);
//         recordDeallocation(blockSize_);
//     }
//
//     void PoolAllocator::reset() {
//         // Reset all blocks to free state
//         usedBlocks_.store(0, std::memory_order_release);
//
// #ifdef _DEBUG
//         // Clear memory with uninitialized pattern
//         std::memset(memory_, 0xCD, blockSize_ * blockCount_);
// #endif
//
//         // Reinitialize free list
//         initializeFreeList();
//
//         // Reset statistics
//         stats.currentUsage.store(0, std::memory_order_release);
//     }
//
//     bool PoolAllocator::owns(const void* ptr) const {
//         if (!ptr || !memory_) return false;
//
//         auto bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);
//         const auto memStart = reinterpret_cast<const std::uint8_t*>(memory_);
//         const std::uint8_t* memEnd = memStart + (blockSize_ * blockCount_);
//
//         // Check if pointer is within pool bounds
//         if (bytePtr < memStart || bytePtr >= memEnd) {
//             return false;
//         }
//
//         // Check if pointer is aligned to block boundary
//         const std::ptrdiff_t offset = bytePtr - memStart;
//         return (offset % blockSize_) == 0;
//     }
//
//     std::size_t PoolAllocator::defragment() {
//         // Pool allocator doesn't fragment in the traditional sense
//         // All blocks are the same size, so no defragmentation needed
//         // This could be used to sort the free list for better cache locality
//
//         // For now, just return 0 (no blocks defragmented)
//         return 0;
//     }

    // ========================================================================
    // LINEAR ALLOCATOR IMPLEMENTATION
    // ========================================================================

//     LinearAllocator::LinearAllocator(MemorySize capacity, const char* name)
//         : memory(nullptr)
//           , capacity(capacity)
//           , current(0)
//           , allocationCount(0)
//           , name(name) {
//         // Allocate backing memory with cache line alignment
//         memory = std::aligned_alloc(CACHE_LINE_SIZE, capacity);
//
//         if (!memory) {
//             throw std::bad_alloc();
//         }
//
//         // Initialize memory with debug pattern
// #ifdef _DEBUG
//         std::memset(memory, 0xCD, capacity);
// #endif
//
//         // Initialize statistics
//         stats.currentUsage = 0;
//         stats.peakUsage = 0;
//         stats.totalAllocated = 0;
//         stats.allocationCount = 0;
//     }
//
//     LinearAllocator::~LinearAllocator() {
//         if (memory) {
// #ifdef _DEBUG
//             if (current.load() > 0) {
//                 std::cerr << "Warning: LinearAllocator '" << name_
//                     << "' destroyed with " << current_.load()
//                     << " bytes still allocated!" << std::endl;
//             }
// #endif
//
//             std::free(memory);
//             memory = nullptr;
//         }
//     }
//
//     LinearAllocator::LinearAllocator(LinearAllocator&& other) noexcept
//         : memory(other.memory)
//           , capacity(other.capacity)
//           , current(other.current.load())
//           , allocationCount(other.allocationCount.load())
//           , name(other.name) {
//         other.memory = nullptr;
//         other.capacity = 0;
//         other.current = 0;
//         other.allocationCount = 0;
//
// #ifdef _DEBUG
//         std::lock_guard<std::mutex> lock(other.debugMutex);
//         debugAllocations = std::move(other.debugAllocations);
// #endif
//     }
//
//     LinearAllocator& LinearAllocator::operator=(LinearAllocator&& other) noexcept {
//         if (this != &other) {
//             // Free existing memory
//             if (memory) {
//                 std::free(memory);
//             }
//
//             // Move from other
//             memory = other.memory;
//             capacity = other.capacity;
//             current = other.current.load();
//             allocationCount = other.allocationCount.load();
//             name = other.name;
//
//             // Reset other
//             other.memory = nullptr;
//             other.capacity = 0;
//             other.current = 0;
//             other.allocationCount = 0;
//
// #ifdef _DEBUG
//             std::lock_guard<std::mutex> lock(other.debugMutex);
//             debugAllocations = std::move(other.debugAllocations);
// #endif
//         }
//         return *this;
//     }
//
//     void* LinearAllocator::allocate(MemorySize size, MemorySize alignment, AllocationFlags flags) {
//         // Validate alignment
//         assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");
//
//         // Get current position
//         MemorySize currentPos = current.load(std::memory_order_acquire);
//
//         // Calculate aligned address
//         void* currentAddress = reinterpret_cast<std::uint8_t*>(memory) + currentPos;
//         void* alignedAddress = alignPointer(currentAddress, alignment);
//
//         // Calculate adjustment for alignment
//         const MemorySize adjustment = reinterpret_cast<std::uint8_t*>(alignedAddress) -
//             reinterpret_cast<std::uint8_t*>(currentAddress);
//
//         // Calculate new position
//         MemorySize newPos = currentPos + adjustment + size;
//
//         // Check if allocation fits
//         if (newPos > capacity) {
//             stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//             return nullptr;
//         }
//
//         // Atomic update of current position
//         MemorySize expected = currentPos;
//         while (!current.compare_exchange_weak(expected, newPos,
//                                                std::memory_order_acq_rel,
//                                                std::memory_order_acquire)) {
//             // Recalculate with new current position
//             currentPos = expected;
//             currentAddress = reinterpret_cast<std::uint8_t*>(memory) + currentPos;
//             alignedAddress = alignPointer(currentAddress, alignment);
//             const MemorySize newAdjustment = reinterpret_cast<std::uint8_t*>(alignedAddress) -
//                 reinterpret_cast<std::uint8_t*>(currentAddress);
//             const MemorySize newNewPos = currentPos + newAdjustment + size;
//
//             if (newNewPos > capacity) {
//                 stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//                 return nullptr;
//             }
//
//             newPos = newNewPos;
//         }
//
//         // Increment allocation count
//         allocationCount.fetch_add(1, std::memory_order_relaxed);
//
// #ifdef _DEBUG
//         // Track allocation for debugging
//         {
//             std::lock_guard<std::mutex> lock(debugMutex);
//             debugAllocations.push_back({currentPos + adjustment, size});
//         }
// #endif
//
//         // Handle allocation flags
//         if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
//             std::memset(alignedAddress, 0, size);
//         }
// #ifdef _DEBUG
//         else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
//             std::memset(alignedAddress, 0xAB, size);
//         }
// #endif
//
//         // Update statistics
//         recordAllocation(size + adjustment);
//
//         return alignedAddress;
//     }
//
//     void LinearAllocator::deallocate(void* ptr) {
//         // Linear allocator doesn't support individual deallocation
//         // This is a no-op
// #ifdef _DEBUG
//         if (ptr) {
//             // In debug mode, warn about attempted deallocation
//             static bool warningShown = false;
//             if (!warningShown) {
//                 std::cerr << "Warning: LinearAllocator does not support individual deallocation. "
//                     << "Use reset() to free all memory." << std::endl;
//                 warningShown = true;
//             }
//         }
// #endif
//     }
//
//     void LinearAllocator::reset() {
// #ifdef _DEBUG
//         // Fill freed memory with pattern
//         MemorySize used = current.load(std::memory_order_acquire);
//         if (used > 0) {
//             std::memset(memory, 0xCD, used);
//         }
//
//         // Clear debug allocations
//         {
//             std::lock_guard<std::mutex> lock(debugMutex);
//             debugAllocations.clear();
//         }
// #endif
//
//         // Reset to beginning
//         current.store(0, std::memory_order_release);
//         allocationCount.store(0, std::memory_order_release);
//
//         // Update statistics
//         stats.currentUsage.store(0, std::memory_order_release);
//     }
//
//     bool LinearAllocator::owns(const void* ptr) const {
//         if (!ptr || !memory) return false;
//
//         const auto* bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);
//         const std::uint8_t* memStart = reinterpret_cast<const std::uint8_t*>(memory);
//         const std::uint8_t* memEnd = memStart + capacity;
//
//         return bytePtr >= memStart && bytePtr < memEnd;
//     }
//
//     MemorySize LinearAllocator::getAllocationSize(const void* ptr) const {
//         if (!owns(ptr)) return 0;
//
// #ifdef _DEBUG
//         // In debug mode, find the allocation in our tracking list
//         std::lock_guard<std::mutex> lock(debugMutex);
//
//         const std::uint8_t* bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);
//         const std::uint8_t* memStart = reinterpret_cast<const std::uint8_t*>(memory);
//         const MemorySize ptrOffset = bytePtr - memStart;
//
//         for (const auto& alloc : debugAllocations) {
//             if (alloc.offset == ptrOffset) {
//                 return alloc.size;
//             }
//         }
// #endif
//
//         // Can't determine size without debug info
//         return 0;
//     }

    // ========================================================================
    // RING BUFFER ALLOCATOR IMPLEMENTATION
    // ========================================================================

//     RingBufferAllocator::RingBufferAllocator(MemorySize capacity, const char* name)
//         : memory(nullptr)
//           , capacity(capacity)
//           , head(0)
//           , tail(0)
//           , fenceCounter(0)
//           , name(name) {
//         // Allocate backing memory with cache line alignment
//         memory = std::aligned_alloc(CACHE_LINE_SIZE, capacity);
//
//         if (!memory) {
//             throw std::bad_alloc();
//         }
//
//         // Initialize memory with debug pattern
// #ifdef _DEBUG
//         std::memset(memory, 0xCD, capacity);
// #endif
//
//         // Initialize statistics
//         stats.currentUsage = 0;
//         stats.peakUsage = 0;
//         stats.totalAllocated = 0;
//         stats.allocationCount = 0;
//     }
//
//     RingBufferAllocator::~RingBufferAllocator() {
//         if (memory) {
// #ifdef _DEBUG
//             MemorySize used = getUsedMemory();
//             if (used > 0) {
//                 std::cerr << "Warning: RingBufferAllocator '" << name
//                     << "' destroyed with " << used
//                     << " bytes still allocated!" << std::endl;
//             }
// #endif
//
//             std::free(memory);
//             memory = nullptr;
//         }
//     }
//
//     RingBufferAllocator::RingBufferAllocator(RingBufferAllocator&& other) noexcept
//         : memory(other.memory)
//           , capacity(other.capacity)
//           , head(other.head.load())
//           , tail(other.tail.load())
//           , fenceCounter(other.fenceCounter.load())
//           , name(other.name) {
//         other.memory = nullptr;
//         other.capacity = 0;
//         other.head = 0;
//         other.tail = 0;
//         other.fenceCounter = 0;
//     }
//
//     RingBufferAllocator& RingBufferAllocator::operator=(RingBufferAllocator&& other) noexcept {
//         if (this != &other) {
//             // Free existing memory
//             if (memory) {
//                 std::free(memory);
//             }
//
//             // Move from other
//             memory = other.memory;
//             capacity = other.capacity;
//             head = other.head.load();
//             tail = other.tail.load();
//             fenceCounter = other.fenceCounter.load();
//             name = other.name;
//
//             // Reset other
//             other.memory = nullptr;
//             other.capacity = 0;
//             other.head = 0;
//             other.tail = 0;
//             other.fenceCounter = 0;
//         }
//         return *this;
//     }
//
//     void* RingBufferAllocator::allocate(MemorySize size, MemorySize alignment, AllocationFlags flags) {
//         // Lock for thread safety (ring buffer needs serialized allocation)
//         std::lock_guard lock(allocationMutex);
//
//         // Validate alignment
//         assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");
//
//         // Calculate total size including header
//         const MemorySize headerSize = sizeof(AllocationHeader);
//         const MemorySize alignedHeaderSize = alignSize(headerSize, alignment);
//         const MemorySize totalSize = alignedHeaderSize + size;
//
//         // Get current head and tail positions
//         MemorySize currentHead = head.load(std::memory_order_acquire);
//         MemorySize currentTail = tail.load(std::memory_order_acquire);
//
//         // Calculate available space
//         MemorySize available;
//         if (currentHead >= currentTail) {
//             // Head is ahead of tail (normal case)
//             available = capacity - currentHead + currentTail;
//         }
//         else {
//             // Head wrapped around
//             available = currentTail - currentHead;
//         }
//
//         // Check if we have enough space
//         if (totalSize > available) {
//             // Try to free old allocations by moving tail forward
//             // In a real implementation, this would check fence values
//             stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//             return nullptr;
//         }
//
//         // Calculate new head position
//         MemorySize newHead = currentHead + totalSize;
//
//         // Handle wrap-around
//         if (newHead >= capacity) {
//             // Check if we can wrap
//             if (totalSize > currentTail) {
//                 // Can't wrap - not enough space at beginning
//                 stats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//                 return nullptr;
//             }
//
//             // Wrap to beginning
//             currentHead = 0;
//             newHead = totalSize;
//         }
//
//         // Write allocation header
//         void* headerPtr = reinterpret_cast<std::uint8_t*>(memory) + currentHead;
//         auto* header = reinterpret_cast<AllocationHeader*>(headerPtr);
//         header->size = size;
//         header->fence = fenceCounter.load(std::memory_order_acquire);
// #ifdef _DEBUG
//         header->magic = MAGIC_NUMBER;
// #endif
//
//         // Get user pointer (after header)
//         void* userPtr = reinterpret_cast<std::uint8_t*>(headerPtr) + alignedHeaderSize;
//
//         // Update head position
//         head.store(newHead % capacity, std::memory_order_release);
//
//         // Handle allocation flags
//         if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
//             std::memset(userPtr, 0, size);
//         }
// #ifdef _DEBUG
//         else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
//             std::memset(userPtr, 0xAB, size);
//         }
// #endif
//
//         // Update statistics
//         recordAllocation(totalSize);
//
//         return userPtr;
//     }
//
//     void RingBufferAllocator::deallocate(void* ptr) {
//         // Ring buffer doesn't support individual deallocation
//         // Memory is freed when the tail advances past it
// #ifdef _DEBUG
//         if (ptr) {
//             static bool warningShown = false;
//             if (!warningShown) {
//                 std::cerr << "Warning: RingBufferAllocator does not support individual deallocation. "
//                     << "Use fence system or reset()." << std::endl;
//                 warningShown = true;
//             }
//         }
// #endif
//     }
//
//     MemorySize RingBufferAllocator::getUsedMemory() const {
//         MemorySize currentHead = head.load(std::memory_order_acquire);
//         MemorySize currentTail = tail.load(std::memory_order_acquire);
//
//         if (currentHead >= currentTail) {
//             return currentHead - currentTail;
//         }
//         else {
//             return (capacity - currentTail) + currentHead;
//         }
//     }
//
//     void RingBufferAllocator::reset() {
//         std::lock_guard<std::mutex> lock(allocationMutex);
//
// #ifdef _DEBUG
//         // Fill memory with uninitialized pattern
//         std::memset(memory, 0xCD, capacity);
// #endif
//
//         // Reset positions
//         head.store(0, std::memory_order_release);
//         tail.store(0, std::memory_order_release);
//         fenceCounter.store(0, std::memory_order_release);
//
//         // Reset statistics
//         stats.currentUsage.store(0, std::memory_order_release);
//     }
//
//     bool RingBufferAllocator::owns(const void* ptr) const {
//         if (!ptr || !memory) return false;
//
//         const std::uint8_t* bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);
//         const std::uint8_t* memStart = reinterpret_cast<const std::uint8_t*>(memory);
//         const std::uint8_t* memEnd = memStart + capacity;
//
//         return bytePtr >= memStart && bytePtr < memEnd;
//     }
//
//     MemorySize RingBufferAllocator::getAllocationSize(const void* ptr) const {
//         if (!owns(ptr)) return 0;
//
// #ifdef _DEBUG
//         // Try to find the header before this allocation
//         const std::uint8_t* bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);
//
//         // Search backwards for the header (within reasonable distance)
//         for (MemorySize offset = sizeof(AllocationHeader); offset <= 256; offset += alignof(AllocationHeader)) {
//             const AllocationHeader* header = reinterpret_cast<const AllocationHeader*>(bytePtr - offset);
//
//             if (header->magic == MAGIC_NUMBER) {
//                 return header->size;
//             }
//         }
// #endif
//
//         return 0;
//     }
//
//     std::uint64_t RingBufferAllocator::createFence() {
//         return fenceCounter.fetch_add(1, std::memory_order_acq_rel) + 1;
//     }
//
//     void RingBufferAllocator::waitForFence(std::uint64_t fence) {
//         std::lock_guard<std::mutex> lock(allocationMutex);
//
//         // Move tail forward to the position marked by the fence
//         // This would free all allocations before the fence
//         // Implementation depends on how fences are tracked with allocations
//
//         // For now, this is a simplified implementation
//         // In production, you'd track fence values with each allocation
//     }
//
//     bool RingBufferAllocator::canAllocateWithoutWrap(MemorySize size) const {
//         MemorySize currentHead = head.load(std::memory_order_acquire);
//         return (currentHead + size) <= capacity;
//     }

    // ========================================================================
    // MEMORY MANAGER IMPLEMENTATION
    // ========================================================================
//
//     MemoryManager::~MemoryManager() {
//         if (initialized) {
//             shutdown();
//         }
//     }
//
//     bool MemoryManager::initialize(const MemoryManagerConfig& requestedConfig) {
//         if (initialized) {
//             std::cerr << "Warning: MemoryManager already initialized!" << std::endl;
//             return false;
//         }
//
//         config = requestedConfig;
//
//         try {
//             // Create main heap allocator
//             // For simplicity, using a large linear allocator as heap
//             // In production, use a more sophisticated heap allocator (dlmalloc, jemalloc, etc.)
//             mainHeap = std::make_unique<LinearAllocator>(config.mainHeapSize, "MainHeap");
//
// #ifdef _DEBUG
//             // Create debug heap for debug allocations
//             if (config.enableLeakDetection || config.enableBoundsChecking) {
//                 debugHeap_ = std::make_unique<LinearAllocator>(config.debugHeapSize, "DebugHeap");
//             }
// #endif
//
//             // Create frame allocators (multi-buffered)
//             frameAllocators.resize(config.frameBufferCount);
//             for (std::uint8_t i = 0; i < config.frameBufferCount; ++i) {
//                 frameAllocators[i].stackAllocator = std::make_unique<StackAllocator>(
//                     config.frameStackSize,
//                     ("FrameStack_" + std::to_string(i)).c_str()
//                 );
//                 frameAllocators[i].linearAllocator = std::make_unique<LinearAllocator>(
//                     config.frameLinearSize,
//                     ("FrameLinear_" + std::to_string(i)).c_str()
//                 );
//                 frameAllocators[i].frameNumber = 0;
//             }
//
//             // Create specialized allocators for different categories
//
//             // Rendering pool - for render commands, vertex data, etc.
//             if (config.renderingPoolSize > 0) {
//                 // Create multiple pools for different sized render objects
//                 auto renderPool = std::make_unique<PoolAllocator>(
//                     1024, // 1KB blocks for render commands
//                     config.renderingPoolSize / 1024,
//                     DEFAULT_ALIGNMENT,
//                     "RenderPool"
//                 );
//                 categoryAllocators[static_cast<std::size_t>(MemoryCategory::RENDERING)] = std::move(renderPool);
//             }
//
//             // Physics pool - for rigid bodies, colliders, etc.
//             if (config.physicsPoolSize > 0) {
//                 auto physicsPool = std::make_unique<PoolAllocator>(
//                     512, // 512 byte blocks for physics objects
//                     config.physicsPoolSize / 512,
//                     DEFAULT_ALIGNMENT,
//                     "PhysicsPool"
//                 );
//                 categoryAllocators[static_cast<std::size_t>(MemoryCategory::PHYSICS)] = std::move(physicsPool);
//             }
//
//             // Audio ring buffer - for streaming audio
//             if (config.audioRingBufferSize > 0) {
//                 auto audioBuffer = std::make_unique<RingBufferAllocator>(
//                     config.audioRingBufferSize,
//                     "AudioRingBuffer"
//                 );
//                 categoryAllocators[static_cast<std::size_t>(MemoryCategory::AUDIO)] = std::move(audioBuffer);
//             }
//
//             // Network buffer - for packet data
//             if (config.networkBufferSize > 0) {
//                 auto networkBuffer = std::make_unique<RingBufferAllocator>(
//                     config.networkBufferSize,
//                     "NetworkBuffer"
//                 );
//                 categoryAllocators[static_cast<std::size_t>(MemoryCategory::NETWORKING)] = std::move(networkBuffer);
//             }
//
//             // Create custom pools from configuration
//             for (const auto& poolConfig : config.customPools) {
//                 auto pool = std::make_unique<PoolAllocator>(
//                     poolConfig.blockSize,
//                     poolConfig.blockCount,
//                     DEFAULT_ALIGNMENT,
//                     ("CustomPool_" + std::to_string(static_cast<int>(poolConfig.category))).c_str()
//                 );
//
//                 if (!categoryAllocators[static_cast<std::size_t>(poolConfig.category)]) {
//                     categoryAllocators[static_cast<std::size_t>(poolConfig.category)] = std::move(pool);
//                 }
//             }
//
//             // Initialize global statistics
//             std::memset(&globalStats, 0, sizeof(globalStats));
//
//             initialized = true;
//
//             // Log initialization
//             std::cout << "MemoryManager initialized successfully:" << std::endl;
//             std::cout << "  Main Heap: " << (config.mainHeapSize / (1024.0 * 1024.0)) << " MB" << std::endl;
//             std::cout << "  Frame Buffers: " << static_cast<int>(config.frameBufferCount) << std::endl;
//             std::cout << "  Frame Stack: " << (config.frameStackSize / (1024.0 * 1024.0)) << " MB" << std::endl;
//             std::cout << "  Frame Linear: " << (config.frameLinearSize / (1024.0 * 1024.0)) << " MB" << std::endl;
//
//             return true;
//         }
//         catch (const std::exception& e) {
//             std::cerr << "Failed to initialize MemoryManager: " << e.what() << std::endl;
//             shutdown();
//             return false;
//         }
//     }
//
//     void MemoryManager::shutdown() {
//         if (!initialized) {
//             return;
//         }
//
// #ifdef _DEBUG
//         // Check for memory leaks
//         std::size_t leakCount = checkForLeaks();
//         if (leakCount > 0) {
//             std::cerr << "Warning: " << leakCount << " memory leaks detected!" << std::endl;
//
//             // Dump leak information
//             for (const auto& [ptr, record] : allocationRecords) {
//                 std::cerr << "  Leaked " << record.size << " bytes from "
//                     << record.file << ":" << record.line << std::endl;
//             }
//         }
// #endif
//
//         // Clear all allocators
//         frameAllocators.clear();
//         for (auto& allocator : categoryAllocators) {
//             allocator.reset();
//         }
//         customAllocators.clear();
//         mainHeap.reset();
//         debugHeap.reset();
//
//         initialized = false;
//
//         std::cout << "MemoryManager shut down." << std::endl;
//     }
//
//     void* MemoryManager::allocate(MemorySize size, MemoryCategory category,
//                                   MemorySize alignment, AllocationFlags flags) {
//         if (!initialized) {
//             std::cerr << "Error: MemoryManager not initialized!" << std::endl;
//             return nullptr;
//         }
//
//         // Select appropriate allocator
//         IAllocator* allocator = selectAllocator(category, size, flags);
//         if (!allocator) {
//             handleAllocationFailure(size, category);
//             return nullptr;
//         }
//
//         // Perform allocation
//         void* ptr = allocator->allocate(size, alignment, flags);
//
//         if (!ptr) {
//             // Try to free some memory and retry
//             if (performMemoryCleanup(size) >= size) {
//                 ptr = allocator->allocate(size, alignment, flags);
//             }
//
//             if (!ptr) {
//                 handleAllocationFailure(size, category);
//                 return nullptr;
//             }
//         }
//
//         // Update global statistics
//         globalStats.totalAllocated.fetch_add(size, std::memory_order_relaxed);
//         globalStats.currentUsage.fetch_add(size, std::memory_order_relaxed);
//         globalStats.allocationCount.fetch_add(1, std::memory_order_relaxed);
//         globalStats.categoryUsage[static_cast<std::size_t>(category)].fetch_add(size, std::memory_order_relaxed);
//
//         // Update peak usage
//         MemorySize current = globalStats.currentUsage.load(std::memory_order_relaxed);
//         MemorySize peak = globalStats.peakUsage.load(std::memory_order_relaxed);
//         while (current > peak && !globalStats.peakUsage.compare_exchange_weak(peak, current)) {
//             // Keep trying
//         }
//
// #ifdef _DEBUG
//         // Record allocation for leak detection
//         if (config.enableLeakDetection) {
//             recordAllocation(ptr, size, category, __FILE__, __LINE__);
//         }
// #endif
//
//         return ptr;
//     }
//
//     void MemoryManager::deallocate(void* ptr, MemoryCategory category) {
//         if (!ptr) return;
//
//         if (!initialized) {
//             std::cerr << "Error: MemoryManager not initialized!" << std::endl;
//             return;
//         }
//
// #ifdef _DEBUG
//         // Validate allocation
//         if (config.enableLeakDetection) {
//             validateAllocation(ptr);
//             removeAllocationRecord(ptr);
//         }
// #endif
//
//         // Find the allocator that owns this pointer
//         IAllocator* allocator = nullptr;
//
//         // Check category allocator first
//         if (categoryAllocators[static_cast<std::size_t>(category)]) {
//             if (categoryAllocators[static_cast<std::size_t>(category)]->owns(ptr)) {
//                 allocator = categoryAllocators[static_cast<std::size_t>(category)].get();
//             }
//         }
//
//         // Check main heap
//         if (!allocator && mainHeap->owns(ptr)) {
//             allocator = mainHeap.get();
//         }
//
//         // Check frame allocators
//         if (!allocator) {
//             for (auto& frame : frameAllocators) {
//                 if (frame.stackAllocator->owns(ptr)) {
//                     allocator = frame.stackAllocator.get();
//                     break;
//                 }
//                 if (frame.linearAllocator->owns(ptr)) {
//                     allocator = frame.linearAllocator.get();
//                     break;
//                 }
//             }
//         }
//
//         if (!allocator) {
//             std::cerr << "Error: Attempted to deallocate unknown pointer!" << std::endl;
//             assert(false && "Invalid deallocation");
//             return;
//         }
//
//         // Get size before deallocation
//         MemorySize size = allocator->getAllocationSize(ptr);
//
//         // Perform deallocation
//         allocator->deallocate(ptr);
//
//         // Update global statistics
//         if (size > 0) {
//             globalStats.totalFreed.fetch_add(size, std::memory_order_relaxed);
//             globalStats.currentUsage.fetch_sub(size, std::memory_order_relaxed);
//             globalStats.freeCount.fetch_add(1, std::memory_order_relaxed);
//             globalStats.categoryUsage[static_cast<std::size_t>(category)].fetch_sub(size, std::memory_order_relaxed);
//         }
//     }
//
//     void* MemoryManager::reallocate(void* ptr, MemorySize newSize,
//                                     MemoryCategory category, MemorySize alignment) {
//         if (!ptr) {
//             return allocate(newSize, category, alignment);
//         }
//
//         if (newSize == 0) {
//             deallocate(ptr, category);
//             return nullptr;
//         }
//
//         // For simplicity, always allocate new and copy
//         // In production, try to resize in-place if possible
//         void* newPtr = allocate(newSize, category, alignment);
//         if (!newPtr) {
//             return nullptr;
//         }
//
//         // Find original size
//         MemorySize oldSize = 0;
//         for (const auto& allocator : categoryAllocators) {
//             if (allocator && allocator->owns(ptr)) {
//                 oldSize = allocator->getAllocationSize(ptr);
//                 break;
//             }
//         }
//
//         if (oldSize == 0 && mainHeap->owns(ptr)) {
//             oldSize = mainHeap->getAllocationSize(ptr);
//         }
//
//         // Copy data
//         if (oldSize > 0) {
//             std::memcpy(newPtr, ptr, std::min(oldSize, newSize));
//         }
//
//         // Free old allocation
//         deallocate(ptr, category);
//
//         return newPtr;
//     }
//
//     StackAllocator& MemoryManager::getFrameStackAllocator() {
//         std::uint8_t index = currentFrameIndex.load(std::memory_order_acquire);
//         return *frameAllocators[index].stackAllocator;
//     }
//
//     LinearAllocator& MemoryManager::getFrameLinearAllocator() {
//         std::uint8_t index = currentFrameIndex.load(std::memory_order_acquire);
//         return *frameAllocators[index].linearAllocator;
//     }
//
//     void MemoryManager::beginFrame(std::uint64_t frameNumber) {
//         // Advance to next frame buffer
//         std::uint8_t currentIndex = currentFrameIndex.load(std::memory_order_acquire);
//         std::uint8_t nextIndex = (currentIndex + 1) % config.frameBufferCount;
//
//         // Reset the allocators we're about to use (from N frames ago)
//         frameAllocators[nextIndex].stackAllocator->reset();
//         frameAllocators[nextIndex].linearAllocator->reset();
//         frameAllocators[nextIndex].frameNumber = frameNumber;
//
//         // Switch to next frame
//         currentFrameIndex.store(nextIndex, std::memory_order_release);
//     }
//
//     void MemoryManager::endFrame() {
//         // Frame cleanup if needed
//         // Most work is done in beginFrame for the next frame
//     }
//
//     IAllocator* MemoryManager::getAllocator(MemoryCategory category) {
//         if (category >= MemoryCategory::COUNT) {
//             return nullptr;
//         }
//
//         return categoryAllocators[static_cast<std::size_t>(category)].get();
//     }
//
//     void MemoryManager::registerAllocator(MemoryCategory category,
//                                           std::unique_ptr<IAllocator> allocator) {
//         if (category >= MemoryCategory::COUNT) {
//             std::cerr << "Error: Invalid memory category!" << std::endl;
//             return;
//         }
//
//         categoryAllocators[static_cast<std::size_t>(category)] = std::move(allocator);
//     }
//
//     MemorySize MemoryManager::getTotalMemoryUsage() const {
//         return globalStats.currentUsage.load(std::memory_order_acquire);
//     }
//
//     MemorySize MemoryManager::getCategoryMemoryUsage(MemoryCategory category) const {
//         if (category >= MemoryCategory::COUNT) {
//             return 0;
//         }
//
//         return globalStats.categoryUsage[static_cast<std::size_t>(category)].load(std::memory_order_acquire);
//     }
//
//     std::string MemoryManager::generateMemoryReport() const {
//         std::stringstream report;
//
//         report << "=== Memory Manager Report ===" << std::endl;
//         report << std::endl;
//
//         // Global statistics
//         report << "Global Statistics:" << std::endl;
//         report << "  Current Usage: " << (getTotalMemoryUsage() / (1024.0 * 1024.0)) << " MB" << std::endl;
//         report << "  Peak Usage: " << (globalStats.peakUsage.load() / (1024.0 * 1024.0)) << " MB" << std::endl;
//         report << "  Total Allocated: " << (globalStats.totalAllocated.load() / (1024.0 * 1024.0)) << " MB" <<
//             std::endl;
//         report << "  Total Freed: " << (globalStats.totalFreed.load() / (1024.0 * 1024.0)) << " MB" << std::endl;
//         report << "  Allocation Count: " << globalStats.allocationCount.load() << std::endl;
//         report << "  Free Count: " << globalStats.freeCount.load() << std::endl;
//         report << "  Failed Allocations: " << globalStats.failedAllocations.load() << std::endl;
//         report << std::endl;
//
//         // Category breakdown
//         report << "Category Usage:" << std::endl;
//         const char* categoryNames[] = {
//             "GENERAL", "RENDERING", "PHYSICS", "AUDIO", "GAMEPLAY",
//             "NETWORKING", "SCRIPTING", "UI", "WORLD", "ANIMATION",
//             "PARTICLES", "AI", "RESOURCES", "DEBUG"
//         };
//
//         for (std::size_t i = 0; i < static_cast<std::size_t>(MemoryCategory::COUNT); ++i) {
//             MemorySize usage = globalStats.categoryUsage[i].load();
//             if (usage > 0) {
//                 report << "  " << std::setw(12) << categoryNames[i] << ": "
//                     << std::setw(10) << (usage / (1024.0 * 1024.0)) << " MB" << std::endl;
//             }
//         }
//         report << std::endl;
//
//         // Allocator details
//         report << "Allocator Details:" << std::endl;
//
//         if (mainHeap) {
//             report << "  Main Heap: "
//                 << (mainHeap->getUsedMemory() / (1024.0 * 1024.0)) << " / "
//                 << (mainHeap->getCapacity() / (1024.0 * 1024.0)) << " MB" << std::endl;
//         }
//
//         // Frame allocators
//         std::uint8_t currentFrame = currentFrameIndex.load();
//         for (std::uint8_t i = 0; i < config.frameBufferCount; ++i) {
//             std::string marker = (i == currentFrame) ? " [CURRENT]" : "";
//             report << "  Frame " << static_cast<int>(i) << " Stack" << marker << ": "
//                 << (frameAllocators[i].stackAllocator->getUsedMemory() / (1024.0 * 1024.0)) << " / "
//                 << (frameAllocators[i].stackAllocator->getCapacity() / (1024.0 * 1024.0)) << " MB" << std::endl;
//             report << "  Frame " << static_cast<int>(i) << " Linear" << marker << ": "
//                 << (frameAllocators[i].linearAllocator->getUsedMemory() / (1024.0 * 1024.0)) << " / "
//                 << (frameAllocators[i].linearAllocator->getCapacity() / (1024.0 * 1024.0)) << " MB" << std::endl;
//         }
//
//         return report.str();
//     }
//
//     bool MemoryManager::dumpAllocations(const std::string& filename) const {
//         std::ofstream file(filename);
//         if (!file.is_open()) {
//             return false;
//         }
//
//         file << generateMemoryReport();
//
// #ifdef _DEBUG
//         if (config.enableLeakDetection) {
//             file << std::endl;
//             file << "=== Active Allocations ===" << std::endl;
//
//             std::shared_lock<std::shared_mutex> lock(recordsMutex);
//             for (const auto& [ptr, record] : allocationRecords) {
//                 file << "Address: " << ptr
//                     << ", Size: " << record.size
//                     << ", Category: " << static_cast<int>(record.category)
//                     << ", File: " << record.file
//                     << ", Line: " << record.line
//                     << std::endl;
//             }
//         }
// #endif
//
//         file.close();
//         return true;
//     }
//
//     std::size_t MemoryManager::checkForLeaks() const {
// #ifdef _DEBUG
//         if (config_.enableLeakDetection) {
//             std::shared_lock<std::shared_mutex> lock(recordsMutex_);
//             return allocationRecords_.size();
//         }
// #endif
//         return 0;
//     }
//
//     void MemoryManager::registerMemoryPressureCallback(MemoryPressureCallback callback) {
//         memoryPressureCallbacks.push_back(callback);
//     }
//
//     MemorySize MemoryManager::performMemoryCleanup(MemorySize targetBytes) {
//         MemorySize freedBytes = 0;
//
//         // Trigger memory pressure callbacks
//         for (const auto& callback : memoryPressureCallbacks) {
//             callback(getTotalMemoryUsage(), targetBytes);
//         }
//
//         // Reset non-critical allocators if needed
//         // This is a simplified implementation
//
//         return freedBytes;
//     }
//
//     void MemoryManager::getSystemMemoryInfo(MemorySize& totalPhysical,
//                                             MemorySize& availablePhysical,
//                                             MemorySize& totalVirtual,
//                                             MemorySize& availableVirtual) {
// #ifdef _WIN32
//         MEMORYSTATUSEX memStatus;
//         memStatus.dwLength = sizeof(memStatus);
//         GlobalMemoryStatusEx(&memStatus);
//
//         totalPhysical = memStatus.ullTotalPhys;
//         availablePhysical = memStatus.ullAvailPhys;
//         totalVirtual = memStatus.ullTotalVirtual;
//         availableVirtual = memStatus.ullAvailVirtual;
//
// #elif __APPLE__
//         int mib[2];
//         size_t length;
//
//         // Get physical memory
//         mib[0] = CTL_HW;
//         mib[1] = HW_MEMSIZE;
//         length = sizeof(totalPhysical);
//         sysctl(mib, 2, &totalPhysical, &length, nullptr, 0);
//
//         // Get available memory (approximation)
//         vm_size_t page_size;
//         vm_statistics64_data_t vm_stat;
//         mach_msg_type_number_t host_size = sizeof(vm_stat) / sizeof(natural_t);
//
//         host_page_size(mach_host_self(), &page_size);
//         host_statistics64(mach_host_self(), HOST_VM_INFO64, (host_info64_t)&vm_stat, &host_size);
//
//         availablePhysical = (vm_stat.free_count + vm_stat.inactive_count) * page_size;
//
//         // Virtual memory info not easily available on macOS
//         totalVirtual = totalPhysical * 2; // Approximation
//         availableVirtual = availablePhysical * 2;
//
// #elif __linux__
//         struct sysinfo memInfo;
//         sysinfo (&memInfo);
//
//         totalPhysical = memInfo.totalram * memInfo.mem_unit;
//         availablePhysical = memInfo.freeram * memInfo.mem_unit;
//         totalVirtual = memInfo.totalswap * memInfo.mem_unit + totalPhysical;
//         availableVirtual = memInfo.freeswap * memInfo.mem_unit + availablePhysical;
//
// #else
//         // Unknown platform
//         totalPhysical = 0;
//         availablePhysical = 0;
//         totalVirtual = 0;
//         availableVirtual = 0;
// #endif
//     }
//
//     IAllocator* MemoryManager::selectAllocator(MemoryCategory category, MemorySize size, AllocationFlags flags) {
//         // Check for thread-local flag
//         if (hasFlags(flags, AllocationFlags::THREAD_LOCAL)) {
//             // Use frame stack for thread-local allocations
//             return &getFrameStackAllocator();
//         }
//
//         // Check for temporary flag
//         if (hasFlags(flags, AllocationFlags::TEMPORARY)) {
//             // Use frame linear for temporary allocations
//             return &getFrameLinearAllocator();
//         }
//
//         // Check category-specific allocator
//         if (category < MemoryCategory::COUNT) {
//             IAllocator* categoryAllocator = categoryAllocators[static_cast<std::size_t>(category)].get();
//             if (categoryAllocator) {
//                 return categoryAllocator;
//             }
//         }
//
//         // Fall back to main heap
//         return mainHeap.get();
//     }
//
//     void MemoryManager::handleAllocationFailure(MemorySize size, MemoryCategory category) {
//         globalStats.failedAllocations.fetch_add(1, std::memory_order_relaxed);
//
//         std::cerr << "Memory allocation failed: " << size << " bytes, category: "
//             << static_cast<int>(category) << std::endl;
//
//         // Try to recover by triggering cleanup
//         MemorySize freed = performMemoryCleanup(size);
//
//         if (freed < size) {
//             // Check if we're in critical memory situation
//             MemorySize available = 0;
//             if (mainHeap) {
//                 available = mainHeap->getFreeMemory();
//             }
//
//             if (available < config.criticalMemoryThreshold) {
//                 std::cerr << "CRITICAL: System is out of memory!" << std::endl;
//
//                 // In production, this might trigger emergency measures
//                 // like unloading non-essential resources
//             }
//         }
//     }
//
// #ifdef _DEBUG
//     void MemoryManager::recordAllocation(void* ptr, MemorySize size, MemoryCategory category,
//                                          const char* file, int line) {
//         AllocationRecord record;
//         record.address = ptr;
//         record.size = size;
//         record.category = category;
//         record.file = file;
//         record.line = line;
//         record.timestamp = std::chrono::steady_clock::now();
//         record.threadId = std::this_thread::get_id();
//
//         std::unique_lock<std::shared_mutex> lock(recordsMutex_);
//         allocationRecords_[ptr] = record;
//     }
//
//     void MemoryManager::removeAllocationRecord(void* ptr) {
//         std::unique_lock<std::shared_mutex> lock(recordsMutex_);
//         allocationRecords_.erase(ptr);
//     }
//
//     void MemoryManager::validateAllocation(void* ptr) const {
//         std::shared_lock<std::shared_mutex> lock(recordsMutex_);
//
//         auto it = allocationRecords_.find(ptr);
//         if (it == allocationRecords_.end()) {
//             std::cerr << "Error: Attempting to free untracked allocation!" << std::endl;
//             assert(false && "Invalid free");
//         }
//     }
// #endif
// } // namespace engine::memory
