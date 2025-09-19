/**
 * @file InputMemoryPools.cpp
 * @brief Memory pool management implementation
 * @author Andrés Guerrero
 * @date 13-09-2025
 */

#include "InputMemoryPools.h"

namespace engine::input {
    // Buffer size constants
    static constexpr std::size_t SMALL_BUFFER_SIZE = 256;
    static constexpr std::size_t MEDIUM_BUFFER_SIZE = 1024;
    static constexpr std::size_t LARGE_BUFFER_SIZE = 4096;
    static constexpr std::size_t STRING_BUFFER_SIZE = sizeof(InputMemoryPools::StringBuffer);

    InputMemoryPools::InputMemoryPools() noexcept {
    }

    InputMemoryPools::~InputMemoryPools() {
        shutdown();
    }

    bool InputMemoryPools::initialize(const InputMemoryPoolConfig& config) {
        if (initialized_) {
            return false;
        }

        config_ = config;

        // Create all pools
        if (!createPools()) {
            destroyPools();
            return false;
        }

        // Reset statistics
        stats_.reset();

        initialized_ = true;
        return true;
    }

    void InputMemoryPools::shutdown() {
        if (!initialized_) {
            return;
        }

        // Destroy all pools
        destroyPools();

        // Clear statistics
        stats_.reset();

        initialized_ = false;
    }

    InputEvent* InputMemoryPools::allocateEvent() noexcept {
        if (!eventPool_) {
            return nullptr;
        }

        void* memory = eventPool_->allocate(sizeof(InputEvent), alignof(InputEvent));
        if (!memory) {
            stats_.eventPoolExhausted.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        // Construct in-place
        const auto event = new(memory) InputEvent();

        // Update statistics
        stats_.eventsAllocated.fetch_add(1, std::memory_order_relaxed);
        stats_.eventsInUse.fetch_add(1, std::memory_order_relaxed);

#ifdef _DEBUG
        if (config_.enableDebugTracking) {
            trackAllocation(event, sizeof(InputEvent), __FILE__, __LINE__);
        }
#endif

        return event;
    }

    void InputMemoryPools::deallocateEvent(InputEvent* event) noexcept {
        if (!event || !eventPool_) {
            return;
        }

        // Check ownership
        if (!eventPool_->owns(event)) {
            // Not from our pool, use regular delete
            delete event;
            return;
        }

#ifdef _DEBUG
        if (config_.enableDebugTracking) {
            untrackAllocation(event);
        }
#endif

        // Destruct and deallocate
        event->~InputEvent();
        eventPool_->deallocate(event);

        // Update statistics
        stats_.eventsInUse.fetch_sub(1, std::memory_order_relaxed);
    }

    InputSnapshot* InputMemoryPools::allocateSnapshot() noexcept {
        if (!snapshotPool_) {
            return nullptr;
        }

        void* memory = snapshotPool_->allocate(sizeof(InputSnapshot), alignof(InputSnapshot));
        if (!memory) {
            stats_.snapshotPoolExhausted.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        // Construct in-place
        const auto snapshot = new(memory) InputSnapshot();

        // Update statistics
        stats_.snapshotsAllocated.fetch_add(1, std::memory_order_relaxed);
        stats_.snapshotsInUse.fetch_add(1, std::memory_order_relaxed);

#ifdef _DEBUG
        if (config_.enableDebugTracking) {
            trackAllocation(snapshot, sizeof(InputSnapshot), __FILE__, __LINE__);
        }
#endif

        return snapshot;
    }

    void InputMemoryPools::deallocateSnapshot(InputSnapshot* snapshot) noexcept {
        if (!snapshot || !snapshotPool_) {
            return;
        }

        // Check ownership
        if (!snapshotPool_->owns(snapshot)) {
            // Not from our pool, use regular delete
            delete snapshot;
            return;
        }

#ifdef _DEBUG
        if (config_.enableDebugTracking) {
            untrackAllocation(snapshot);
        }
#endif

        // Destruct and deallocate
        snapshot->~InputSnapshot();
        snapshotPool_->deallocate(snapshot);

        // Update statistics
        stats_.snapshotsInUse.fetch_sub(1, std::memory_order_relaxed);
    }

    void* InputMemoryPools::allocateBuffer(const std::size_t size) noexcept {
        const BufferSize category = getBufferSizeCategory(size);
        return getBuffer(category);
    }

    void InputMemoryPools::deallocateBuffer(void* buffer, const std::size_t size) noexcept {
        if (!buffer) {
            return;
        }

        const BufferSize category = getBufferSizeCategory(size);
        returnBuffer(buffer, category);
    }

    void* InputMemoryPools::getBuffer(const BufferSize size) noexcept {
        memory::PoolAllocator* pool = nullptr;
        std::size_t bufferSize = 0;

        switch (size) {
        case BufferSize::SMALL:
            pool = smallBufferPool_.get();
            bufferSize = SMALL_BUFFER_SIZE;
            break;
        case BufferSize::MEDIUM:
            pool = mediumBufferPool_.get();
            bufferSize = MEDIUM_BUFFER_SIZE;
            break;
        case BufferSize::LARGE:
            pool = largeBufferPool_.get();
            bufferSize = LARGE_BUFFER_SIZE;
            break;
        }

        if (!pool) {
            return nullptr;
        }

        void* buffer = pool->allocate(bufferSize, 16);
        if (!buffer) {
            stats_.bufferPoolExhausted.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        // Update statistics
        stats_.buffersAllocated.fetch_add(1, std::memory_order_relaxed);
        stats_.buffersInUse.fetch_add(1, std::memory_order_relaxed);

#ifdef _DEBUG
        if (config_.enableDebugTracking) {
            trackAllocation(buffer, bufferSize, __FILE__, __LINE__);
        }
#endif

        return buffer;
    }

    void InputMemoryPools::returnBuffer(void* buffer, const BufferSize size) noexcept {
        if (!buffer) {
            return;
        }

        memory::PoolAllocator* pool = nullptr;

        switch (size) {
        case BufferSize::SMALL:
            pool = smallBufferPool_.get();
            break;
        case BufferSize::MEDIUM:
            pool = mediumBufferPool_.get();
            break;
        case BufferSize::LARGE:
            pool = largeBufferPool_.get();
            break;
        }

        if (!pool || !pool->owns(buffer)) {
            // Not from our pool, use regular free
            std::free(buffer);
            return;
        }

#ifdef _DEBUG
        if (config_.enableDebugTracking) {
            untrackAllocation(buffer);
        }
#endif

        pool->deallocate(buffer);

        // Update statistics
        stats_.buffersInUse.fetch_sub(1, std::memory_order_relaxed);
    }

    InputMemoryPools::StringBuffer* InputMemoryPools::allocateString() const noexcept {
        if (!stringPool_) {
            return nullptr;
        }

        void* memory = stringPool_->allocate(STRING_BUFFER_SIZE, alignof(StringBuffer));
        if (!memory) {
            stats_.stringPoolExhausted.fetch_add(1, std::memory_order_relaxed);
            return nullptr;
        }

        // Construct in-place
        const auto buffer = new(memory) StringBuffer();
        buffer->inUse.store(true, std::memory_order_release);

        // Update statistics
        stats_.stringsAllocated.fetch_add(1, std::memory_order_relaxed);
        stats_.stringsInUse.fetch_add(1, std::memory_order_relaxed);

        return buffer;
    }

    void InputMemoryPools::deallocateString(StringBuffer* buffer) const noexcept {
        if (!buffer || !stringPool_) {
            return;
        }

        // Check ownership
        if (!stringPool_->owns(buffer)) {
            delete buffer;
            return;
        }

        // Mark as not in use
        buffer->inUse.store(false, std::memory_order_release);

        // Clear the buffer
        buffer->data[0] = '\0';

        // Destruct and deallocate
        buffer->~StringBuffer();
        stringPool_->deallocate(buffer);

        // Update statistics
        stats_.stringsInUse.fetch_sub(1, std::memory_order_relaxed);
    }

    void InputMemoryPools::resetAllPools() noexcept {
        std::lock_guard<std::mutex> lock(poolMutex_);

        // Reset all pools
        if (eventPool_) eventPool_->reset();
        if (snapshotPool_) snapshotPool_->reset();
        if (smallBufferPool_) smallBufferPool_->reset();
        if (mediumBufferPool_) mediumBufferPool_->reset();
        if (largeBufferPool_) largeBufferPool_->reset();
        if (stringPool_) stringPool_->reset();

        // Reset usage statistics
        stats_.eventsInUse = 0;
        stats_.snapshotsInUse = 0;
        stats_.buffersInUse = 0;
        stats_.stringsInUse = 0;
        stats_.totalMemoryInUse = 0;

#ifdef _DEBUG
        allocations_.clear();
#endif
    }

    // TODO: Revisar esto si es necesario, indispensable para input manager
    std::size_t InputMemoryPools::defragmentPools() {
        // Pool allocators don't fragment in the traditional sense
        // All blocks are the same size, so no defragmentation needed
        return 0;
    }

    std::size_t InputMemoryPools::getTotalMemoryUsage() const noexcept {
        std::size_t total = 0;

        if (eventPool_) {
            total += eventPool_->getCapacity();
        }
        if (snapshotPool_) {
            total += snapshotPool_->getCapacity();
        }
        if (smallBufferPool_) {
            total += smallBufferPool_->getCapacity();
        }
        if (mediumBufferPool_) {
            total += mediumBufferPool_->getCapacity();
        }
        if (largeBufferPool_) {
            total += largeBufferPool_->getCapacity();
        }
        if (stringPool_) {
            total += stringPool_->getCapacity();
        }

        return total;
    }

    std::size_t InputMemoryPools::getAvailableEvents() const noexcept {
        if (!eventPool_) {
            return 0;
        }
        return eventPool_->getFreeBlockCount();
    }

    std::size_t InputMemoryPools::getAvailableSnapshots() const noexcept {
        if (!snapshotPool_) {
            return 0;
        }
        return snapshotPool_->getFreeBlockCount();
    }

    bool InputMemoryPools::validatePools() const noexcept {
        // Check that all pools are valid
        if (!eventPool_ || !snapshotPool_) {
            return false;
        }

        // Check usage doesn't exceed capacity
        if (stats_.eventsInUse > config_.eventPoolSize) {
            return false;
        }
        if (stats_.snapshotsInUse > config_.snapshotPoolSize) {
            return false;
        }

        return true;
    }

    // ============================================================================
    // Private Methods Implementation
    // ============================================================================

    bool InputMemoryPools::createPools() {
        try {
            // Create event pool
            eventPool_ = std::make_unique<memory::PoolAllocator>(
                sizeof(InputEvent),
                config_.eventPoolSize,
                alignof(InputEvent),
                "InputEventPool"
            );

            // Create snapshot pool
            snapshotPool_ = std::make_unique<memory::PoolAllocator>(
                sizeof(InputSnapshot),
                config_.snapshotPoolSize,
                alignof(InputSnapshot),
                "InputSnapshotPool"
            );

            // Create buffer pools
            smallBufferPool_ = std::make_unique<memory::PoolAllocator>(
                SMALL_BUFFER_SIZE,
                config_.smallBufferPoolSize,
                16,
                "SmallBufferPool"
            );

            mediumBufferPool_ = std::make_unique<memory::PoolAllocator>(
                MEDIUM_BUFFER_SIZE,
                config_.mediumBufferPoolSize,
                16,
                "MediumBufferPool"
            );

            largeBufferPool_ = std::make_unique<memory::PoolAllocator>(
                LARGE_BUFFER_SIZE,
                config_.largeBufferPoolSize,
                16,
                "LargeBufferPool"
            );

            // Create string pool
            stringPool_ = std::make_unique<memory::PoolAllocator>(
                STRING_BUFFER_SIZE,
                config_.stringPoolSize,
                alignof(StringBuffer),
                "StringPool"
            );

            // Update total memory allocated
            stats_.totalMemoryAllocated.store(getTotalMemoryUsage(), std::memory_order_release);

            return true;
        }
        catch (const std::exception&) {
            return false;
        }
    }

    void InputMemoryPools::destroyPools() {
        // Destroy all pools (RAII will handle cleanup)
        eventPool_.reset();
        snapshotPool_.reset();
        smallBufferPool_.reset();
        mediumBufferPool_.reset();
        largeBufferPool_.reset();
        stringPool_.reset();
    }

    void InputMemoryPools::updateMemoryStats() const {
        std::size_t currentUsage = 0;

        // Calculate current memory usage
        if (eventPool_) {
            currentUsage += eventPool_->getUsedMemory();
        }
        if (snapshotPool_) {
            currentUsage += snapshotPool_->getUsedMemory();
        }
        if (smallBufferPool_) {
            currentUsage += smallBufferPool_->getUsedMemory();
        }
        if (mediumBufferPool_) {
            currentUsage += mediumBufferPool_->getUsedMemory();
        }
        if (largeBufferPool_) {
            currentUsage += largeBufferPool_->getUsedMemory();
        }
        if (stringPool_) {
            currentUsage += stringPool_->getUsedMemory();
        }

        // Update current usage
        stats_.totalMemoryInUse.store(currentUsage, std::memory_order_release);

        // Update peak usage
        std::size_t currentPeak = stats_.peakMemoryUsage.load(std::memory_order_acquire);
        while (currentUsage > currentPeak) {
            if (stats_.peakMemoryUsage.compare_exchange_weak(currentPeak, currentUsage,
                                                             std::memory_order_release,
                                                             std::memory_order_acquire)) {
                break;
            }
        }
    }

    InputMemoryPools::BufferSize InputMemoryPools::getBufferSizeCategory(const std::size_t size) noexcept {
        if (size <= SMALL_BUFFER_SIZE) {
            return BufferSize::SMALL;
        }

        if (size <= MEDIUM_BUFFER_SIZE) {
            return BufferSize::MEDIUM;
        }

        return BufferSize::LARGE;
    }

    void InputMemoryPools::trackAllocation([[maybe_unused]] void* ptr,
                                           [[maybe_unused]] std::size_t size,
                                           [[maybe_unused]] const char* file,
                                           [[maybe_unused]] int line) {
#ifdef _DEBUG
        if (!config_.enableDebugTracking) {
            return;
        }

        std::lock_guard<std::mutex> lock(debugMutex_);

        AllocationInfo info;
        info.ptr = ptr;
        info.size = size;
        info.timestamp = std::chrono::steady_clock::now();
        info.file = file ? file : "";
        info.line = line;

        allocations_[ptr] = info;
#endif
    }

    void InputMemoryPools::untrackAllocation([[maybe_unused]] void* ptr) {
#ifdef _DEBUG
        if (!config_.enableDebugTracking) {
            return;
        }

        std::lock_guard<std::mutex> lock(debugMutex_);
        allocations_.erase(ptr);
#endif
    }
} // namespace engine::input
