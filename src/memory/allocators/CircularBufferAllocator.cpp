//
// Created by Andres Guerrero on 19-09-25.
//

#include "CircularBufferAllocator.h"

namespace engine::memory {
    CircularBufferAllocator::CircularBufferAllocator(const MemorySize capacity, const std::size_t maxEntries,
                                                     const char* name)
        : memory_(nullptr)
          , capacity_(capacity)
          , maxEntries_(maxEntries > 0 ? maxEntries : capacity / 64) // Default to 64-byte entries
          , writeHead_(0)
          , readHead_(0)
          , entryCount_(0)
          , overwriteCount_(0)
          , name_(name) {
        // Allocate aligned memory for optimal cache performance
        memory_ = std::aligned_alloc(CACHE_LINE_SIZE, capacity);

        if (!memory_) {
            std::cerr << "[CircularBufferAllocator] Failed to allocate "
                << (static_cast<double>(capacity) / (1024.0 * 1024.0)) << " MB for " << name_ << std::endl;
            throw std::bad_alloc();
        }

        // Initialize entries tracking array
        entries_.resize(maxEntries_);

        // Clear memory with debug pattern
#ifdef _DEBUG
        std::memset(memory_, 0xCD, capacity);
#endif

        // Initialize statistics
        stats_.currentUsage = 0;
        stats_.peakUsage = 0;
        stats_.totalAllocated = 0;
        stats_.allocationCount = 0;
    }

    CircularBufferAllocator::~CircularBufferAllocator() {
        if (!memory_) return;

#ifdef _DEBUG
        if (entryCount_.load() > 0) {
            std::cerr << "Warning: CircularBufferAllocator '" << name_
                << "' destroyed with " << entryCount_.load()
                << " entries still active!" << std::endl;
        }
#endif

        std::free(memory_);
        memory_ = nullptr;
    }

    CircularBufferAllocator::CircularBufferAllocator(CircularBufferAllocator&& other) noexcept: memory_(other.memory_)
        , capacity_(other.capacity_)
        , maxEntries_(other.maxEntries_)
        , writeHead_(other.writeHead_.load())
        , readHead_(other.readHead_.load())
        , entryCount_(other.entryCount_.load())
        , overwriteCount_(other.overwriteCount_.load())
        , name_(other.name_)
        , entries_(std::move(other.entries_)) {
        other.memory_ = nullptr;
        other.capacity_ = 0;
        other.maxEntries_ = 0;
        other.writeHead_ = 0;
        other.readHead_ = 0;
        other.entryCount_ = 0;
        other.overwriteCount_ = 0;
    }

    CircularBufferAllocator& CircularBufferAllocator::operator=(CircularBufferAllocator&& other) noexcept {
        if (this != &other) {
            if (memory_) {
                std::free(memory_);
            }

            memory_ = other.memory_;
            capacity_ = other.capacity_;
            maxEntries_ = other.maxEntries_;
            writeHead_ = other.writeHead_.load();
            readHead_ = other.readHead_.load();
            entryCount_ = other.entryCount_.load();
            overwriteCount_ = other.overwriteCount_.load();
            name_ = other.name_;
            entries_ = std::move(other.entries_);

            other.memory_ = nullptr;
            other.capacity_ = 0;
            other.maxEntries_ = 0;
            other.writeHead_ = 0;
            other.readHead_ = 0;
            other.entryCount_ = 0;
            other.overwriteCount_ = 0;
        }
        return *this;
    }

    void* CircularBufferAllocator::allocate(const MemorySize size, const MemorySize alignment, const AllocationFlags flags) {
        assert(isPowerOfTwo(alignment) && "Alignment must be power of 2");

        // Check if single allocation exceeds buffer capacity
        if (size > capacity_) {
            stats_.failedAllocations.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        std::lock_guard lock(allocationMutex_);

        // Calculate aligned size including entry header
        constexpr MemorySize headerSize = sizeof(EntryHeader);
        const MemorySize alignedHeaderSize = alignSize(headerSize, alignment);
        const MemorySize totalSize = alignedHeaderSize + size;

        MemorySize currentWrite = writeHead_.load(std::memory_order_acquire);

        // Check if we need to wrap around
        if (currentWrite + totalSize > capacity_) {
            // Wrap to beginning
            currentWrite = 0;
            writeHead_.store(0, std::memory_order_release);
        }

        // Check if we would overwrite unread data
        if (const MemorySize currentRead = readHead_.load(std::memory_order_acquire); currentWrite <= currentRead && currentWrite + totalSize > currentRead) {
            // We're about to overwrite unread data - advance read head
            advanceReadHead(totalSize);
            overwriteCount_.fetch_add(1, std::memory_order_relaxed);
        }

        // Create allocation header
        void* headerPtr = static_cast<std::uint8_t*>(memory_) + currentWrite;
        auto* header = static_cast<EntryHeader*>(headerPtr);
        header->size = size;
        header->totalSize = totalSize;
        header->timestamp = std::chrono::steady_clock::now();
        header->entryId = entryCount_.load(std::memory_order_acquire);
#ifdef _DEBUG
        header->magic = MAGIC_NUMBER;
#endif

        // Calculate user data pointer
        void* userPtr = static_cast<std::uint8_t*>(headerPtr) + alignedHeaderSize;

        // Update write head
        writeHead_.store((currentWrite + totalSize) % capacity_, std::memory_order_release);

        // Track entry
        const std::size_t entryIndex = entryCount_.fetch_add(1, std::memory_order_acq_rel) % maxEntries_;
        entries_[entryIndex] = {currentWrite, totalSize, header->entryId, true};

        // Initialize memory based on flags
        if (hasFlags(flags, AllocationFlags::ZERO_MEMORY)) {
            std::memset(userPtr, 0, size);
        }
#ifdef _DEBUG
            else if (hasFlags(flags, AllocationFlags::DEBUG_FILL)) {
            std::memset(userPtr, 0xAB, size);
        }
#endif

        recordAllocation(totalSize);

        return userPtr;
    }

    void CircularBufferAllocator::deallocate(void* ptr) {
        // Circular buffer doesn't support individual deallocation
        // Memory is automatically reclaimed when buffer wraps
#ifdef _DEBUG
        if (ptr) {
            static bool warningShown = false;
            if (!warningShown) {
                std::cerr << "Warning: CircularBufferAllocator does not support individual deallocation. "
                    << "Memory is automatically reclaimed." << std::endl;
                warningShown = true;
            }
        }
#endif
    }

    MemorySize CircularBufferAllocator::getUsedMemory() const {
        const MemorySize write = writeHead_.load(std::memory_order_acquire);

        if (const MemorySize read = readHead_.load(std::memory_order_acquire); write >= read) {
            return write - read;
        }
        else {
            return (capacity_ - read) + write;
        }
    }

    void CircularBufferAllocator::reset() {
        std::lock_guard lock(allocationMutex_);

#ifdef _DEBUG
        std::memset(memory_, 0xCD, capacity_);
#endif

        writeHead_.store(0, std::memory_order_release);
        readHead_.store(0, std::memory_order_release);
        entryCount_.store(0, std::memory_order_release);
        overwriteCount_.store(0, std::memory_order_release);

        // Clear entries tracking
        for (auto& entry : entries_) {
            entry.valid = false;
        }

        stats_.currentUsage.store(0, std::memory_order_release);
    }

    bool CircularBufferAllocator::owns(const void* ptr) const {
        if (!ptr || !memory_) return false;

        const auto* bytePtr = static_cast<const std::uint8_t*>(ptr);
        const auto* memStart = static_cast<const std::uint8_t*>(memory_);
        const std::uint8_t* memEnd = memStart + capacity_;

        return bytePtr >= memStart && bytePtr < memEnd;
    }

    MemorySize CircularBufferAllocator::getAllocationSize(const void* ptr) const {
        if (!owns(ptr)) return 0;

#ifdef _DEBUG
        // Try to find the header before this allocation
        const std::uint8_t* bytePtr = reinterpret_cast<const std::uint8_t*>(ptr);
        const std::uint8_t* memStart = reinterpret_cast<const std::uint8_t*>(memory_);

        // Search backwards for header within reasonable distance
        for (MemorySize offset = sizeof(EntryHeader); offset <= 256; offset += alignof(EntryHeader)) {
            if (bytePtr - offset < memStart) break;

            const EntryHeader* header = reinterpret_cast<const EntryHeader*>(bytePtr - offset);
            if (header->magic == MAGIC_NUMBER) {
                return header->size;
            }
        }
#endif
        return 0;
    }

    void* CircularBufferAllocator::getOldestEntry() {
        std::lock_guard lock(allocationMutex_);

        if (getUsedMemory() == 0) return nullptr;

        const MemorySize currentRead = readHead_.load(std::memory_order_acquire);
        void* headerPtr = static_cast<std::uint8_t*>(memory_) + currentRead;

#ifdef _DEBUG
        auto* header = static_cast<EntryHeader*>(headerPtr);
        assert(header->magic == MAGIC_NUMBER && "Corrupted entry header");
#endif

        constexpr MemorySize headerSize = sizeof(EntryHeader);
        const MemorySize alignedHeaderSize = alignSize(headerSize, DEFAULT_ALIGNMENT);

        return static_cast<std::uint8_t*>(headerPtr) + alignedHeaderSize;
    }

    void CircularBufferAllocator::advanceReadHead() {
        std::lock_guard lock(allocationMutex_);
        advanceReadHead(0);
    }

    void CircularBufferAllocator::advanceReadHead(const MemorySize minAdvance) {
        const MemorySize currentRead = readHead_.load(std::memory_order_acquire);

        if (minAdvance == 0) {
            // Advance to next entry
            void* headerPtr = static_cast<std::uint8_t*>(memory_) + currentRead;
            auto* header = static_cast<EntryHeader*>(headerPtr);

#ifdef _DEBUG
            assert(header->magic == MAGIC_NUMBER && "Corrupted entry header");
#endif

            const MemorySize newRead = (currentRead + header->totalSize) % capacity_;
            readHead_.store(newRead, std::memory_order_release);
        }
        else {
            // Advance by at least minAdvance bytes
            const MemorySize newRead = (currentRead + minAdvance) % capacity_;
            readHead_.store(newRead, std::memory_order_release);
        }
    }
} // namespace engine::memory
